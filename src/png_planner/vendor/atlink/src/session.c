/*
 * 机载端会话（AT-Link v3）：过滤、序号窗口、幂等、限频、周期回落、失联。
 * 时钟由调用方传入。上行帧经 tx 回调交还，本文件不碰传输。
 */
#include "session.h"
#include "frame.h"
#include "payload.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ATLINK_IDEMP_CAP 16
#define ATLINK_SEQ_WINDOW 32
#define ATLINK_TICK_MS 250u

#define IDEMP_EMPTY   0
#define IDEMP_PENDING 1
#define IDEMP_DONE    2

typedef struct {
    uint16_t req_id;
    uint16_t cmd_id;
    uint8_t  result;
    uint8_t  state;
    uint64_t since_ms;
} idemp_t;

typedef struct {
    int      synced;
    uint8_t  expect;
    uint32_t window_drop;
} seq_state_t;

struct atlink_session {
    uint16_t local_addr;
    uint64_t now_ms;
    uint64_t origin_ms;
    int      have_origin;

    atlink_tx_fn tx;
    void        *tx_user;

    atlink_decoder_t dec;
    seq_state_t      down_seq;
    uint8_t          up_seq;

    idemp_t idemp[ATLINK_IDEMP_CAP];
    int     idemp_i;

    int      takeoff_rate_valid;
    int      period_rate_valid;
    uint64_t last_takeoff_ms;
    uint64_t last_period_ms;

    int      have_down;
    uint64_t last_down_ms;
    int      tick_started;
    uint64_t last_tick_ms;
    uint32_t rx_ok;
    uint32_t tx_ok;
    uint32_t dir_fail;

    atlink_status_t status;
    uint16_t        period_ms;
    int             have_mission;
    atlink_takeoff_t mission;

    int      takeoff_pending;
    uint16_t takeoff_req;
    uint64_t takeoff_since_ms;

    int      link_lost;
    int      link_lost_notify;
    int      timeout_pending;
    uint16_t timeout_req;

    int      telem_win_open;
    uint16_t telem_req_id;

    atlink_vision_t vision;

    atlink_hooks_t hooks;
};

static void vision_reset(atlink_vision_t *v)
{
    memset(v, 0, sizeof(*v));
    v->flight_phase_ext = ATLINK_PHASE_EXT_UNKNOWN;
    v->x1 = ATLINK_BBOX_NONE;
    v->y1 = ATLINK_BBOX_NONE;
    v->x2 = ATLINK_BBOX_NONE;
    v->y2 = ATLINK_BBOX_NONE;
    v->track_id = ATLINK_TRACK_INVALID;
    v->class_id = ATLINK_CLASS_UNKNOWN;
}

static void slog(atlink_session_t *s, int level, const char *fmt, ...)
{
    char buf[256];
    va_list ap;

    if (!s->hooks.log) {
        return;
    }
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    s->hooks.log(s->hooks.user, level, buf);
}

static void note_time(atlink_session_t *s, uint64_t now_ms)
{
    s->now_ms = now_ms;
    if (!s->have_origin) {
        s->have_origin = 1;
        s->origin_ms = now_ms;
    }
}

static int phase_air(uint8_t phase)
{
    return phase == ATLINK_PHASE_TAKING_OFF || phase == ATLINK_PHASE_INTERCEPTING ||
           phase == ATLINK_PHASE_RETURNING || phase == ATLINK_PHASE_LANDING;
}

static int phase_done(uint8_t phase)
{
    return phase == ATLINK_PHASE_STANDBY || phase == ATLINK_PHASE_LANDED;
}

static int phase_intercept(uint8_t phase)
{
    return phase == ATLINK_PHASE_TAKING_OFF || phase == ATLINK_PHASE_INTERCEPTING;
}

static idemp_t *idemp_find(atlink_session_t *s, uint16_t req_id)
{
    int i;

    if (req_id == 0) {
        return NULL;
    }
    for (i = 0; i < ATLINK_IDEMP_CAP; i++) {
        if (s->idemp[i].state != IDEMP_EMPTY && s->idemp[i].req_id == req_id) {
            return &s->idemp[i];
        }
    }
    return NULL;
}

static idemp_t *idemp_slot(atlink_session_t *s, uint16_t req_id)
{
    idemp_t *e = idemp_find(s, req_id);

    if (e) {
        return e;
    }
    e = &s->idemp[s->idemp_i];
    s->idemp_i = (s->idemp_i + 1) % ATLINK_IDEMP_CAP;
    memset(e, 0, sizeof(*e));
    e->req_id = req_id;
    return e;
}

static int send_frame(atlink_session_t *s, uint16_t cmd, uint16_t req_id,
                      const uint8_t *payload, uint16_t plen)
{
    atlink_frame_t f;
    uint8_t buf[ATLINK_MAX_FRAME];
    int n;
    int rc;

    if (!s->tx) {
        return -1;
    }
    memset(&f, 0, sizeof(f));
    f.version = ATLINK_VERSION;
    f.seq = s->up_seq++;
    f.dir = ATLINK_DIR_UP;
    f.addr = s->local_addr;
    f.req_id = req_id;
    f.cmd_id = cmd;
    f.len = plen;
    if (plen && payload) {
        memcpy(f.payload, payload, plen);
    }
    n = atlink_encode(buf, sizeof(buf), &f);
    if (n < 0) {
        return -1;
    }
    rc = s->tx(s->tx_user, buf, (uint16_t)n);
    if (rc == 0) {
        s->tx_ok++;
    }
    return rc;
}

static int send_ack(atlink_session_t *s, uint16_t cmd, uint16_t req_id, uint8_t result, const char *note)
{
    uint8_t pl[6 + ATLINK_ACK_NOTE_MAX];
    int n = atlink_pack_ack(pl, sizeof(pl), cmd, req_id, result, note);
    idemp_t *e;

    if (n < 0) {
        return -1;
    }
    e = idemp_slot(s, req_id);
    e->cmd_id = cmd;
    e->result = result;
    e->state = IDEMP_DONE;
    return send_frame(s, ATLINK_CMD_ACK, req_id, pl, (uint16_t)n);
}

static int send_ack_transient(atlink_session_t *s, uint16_t cmd, uint16_t req_id,
                              uint8_t result, const char *note)
{
    uint8_t pl[6 + ATLINK_ACK_NOTE_MAX];
    int n = atlink_pack_ack(pl, sizeof(pl), cmd, req_id, result, note);

    if (n < 0) {
        return -1;
    }
    return send_frame(s, ATLINK_CMD_ACK, req_id, pl, (uint16_t)n);
}

static void mark_pending(atlink_session_t *s, uint16_t req_id, uint16_t cmd)
{
    idemp_t *e = idemp_slot(s, req_id);

    e->cmd_id = cmd;
    e->state = IDEMP_PENDING;
    e->result = 0;
    e->since_ms = s->now_ms;
}

static int finish_takeoff(atlink_session_t *s, uint8_t result, const char *note)
{
    uint16_t req;

    if (!s->takeoff_pending) {
        return -1;
    }
    req = s->takeoff_req;
    s->takeoff_pending = 0;
    return send_ack(s, ATLINK_CMD_TAKEOFF, req, result, note);
}

static int filter_downlink(const atlink_frame_t *f, uint16_t local_addr)
{
    if (!f || f->version != ATLINK_VERSION || f->dir != ATLINK_DIR_DOWN) {
        return -1;
    }
    if (f->addr != local_addr && f->addr != ATLINK_ADDR_BROADCAST) {
        return 0;
    }
    return 1;
}

static int broadcast_allowed(uint16_t cmd_id)
{
    return cmd_id == ATLINK_CMD_ABORT || cmd_id == ATLINK_CMD_PERIOD;
}

static int seq_accept(seq_state_t *st, uint8_t seq, int broadcast)
{
    uint8_t diff;

    if (!st) {
        return 0;
    }
    if (broadcast) {
        return 1;
    }
    if (!st->synced) {
        st->synced = 1;
        st->expect = (uint8_t)(seq + 1u);
        return 1;
    }
    diff = (uint8_t)(seq - st->expect);
    if (diff > ATLINK_SEQ_WINDOW && diff < (uint8_t)(256 - ATLINK_SEQ_WINDOW)) {
        st->window_drop++;
        return 0;
    }
    st->expect = (uint8_t)(seq + 1u);
    return 1;
}

static int rate_ok(const atlink_session_t *s, uint16_t cmd)
{
    if (cmd == ATLINK_CMD_TAKEOFF) {
        return !(s->takeoff_rate_valid && s->now_ms - s->last_takeoff_ms < ATLINK_CMD_RATE_MS);
    }
    if (cmd == ATLINK_CMD_PERIOD) {
        return !(s->period_rate_valid && s->now_ms - s->last_period_ms < ATLINK_CMD_RATE_MS);
    }
    return 1;
}

static void rate_stamp(atlink_session_t *s, uint16_t cmd)
{
    if (cmd == ATLINK_CMD_TAKEOFF) {
        s->takeoff_rate_valid = 1;
        s->last_takeoff_ms = s->now_ms;
    } else if (cmd == ATLINK_CMD_PERIOD) {
        s->period_rate_valid = 1;
        s->last_period_ms = s->now_ms;
    }
}

static uint64_t silence_ms(const atlink_session_t *s)
{
    uint64_t ref = s->have_down ? s->last_down_ms : s->origin_ms;

    if (!s->have_origin || s->now_ms < ref) {
        return 0;
    }
    return s->now_ms - ref;
}

static void deliver_async(atlink_session_t *s)
{
    atlink_on_takeoff_timeout_fn on_to;
    atlink_on_link_lost_fn on_lost;
    void *ud;
    uint16_t req;

    if (s->timeout_pending) {
        s->timeout_pending = 0;
        req = s->timeout_req;
        on_to = s->hooks.on_takeoff_timeout;
        ud = s->hooks.user;
        if (on_to) {
            on_to(req, ud);
        }
    }
    if (s->link_lost_notify) {
        s->link_lost_notify = 0;
        on_lost = s->hooks.on_link_lost;
        ud = s->hooks.user;
        if (on_lost) {
            on_lost(ud);
        }
    }
}

static void eval_timeouts(atlink_session_t *s)
{
    uint64_t t = s->now_ms;
    uint64_t quiet;
    uint16_t stale_req[ATLINK_IDEMP_CAP];
    uint16_t stale_cmd[ATLINK_IDEMP_CAP];
    int n = 0;
    int i;

    if (s->tick_started && t - s->last_tick_ms < ATLINK_TICK_MS) {
        return;
    }
    s->tick_started = 1;
    s->last_tick_ms = t;
    quiet = silence_ms(s);

    if (s->period_ms != ATLINK_PERIOD_DEFAULT_MS && quiet >= ATLINK_LINK_LOST_MS) {
        s->period_ms = ATLINK_PERIOD_DEFAULT_MS;
        slog(s, 1, "report period fallback to 1000 ms");
    }
    if (!s->link_lost && phase_air(s->status.flight_phase) && quiet >= ATLINK_LINK_LOST_MS) {
        s->link_lost = 1;
        s->link_lost_notify = 1;
        slog(s, 1, "link lost, suggest return");
    }

    if (s->takeoff_pending && s->takeoff_since_ms &&
        t - s->takeoff_since_ms >= ATLINK_TAKEOFF_TIMEOUT_MS) {
        s->timeout_req = s->takeoff_req;
        finish_takeoff(s, ATLINK_RESULT_FAILED, "TIMEOUT");
        s->timeout_pending = 1;
        slog(s, 1, "takeoff timeout");
    }

    for (i = 0; i < ATLINK_IDEMP_CAP; i++) {
        idemp_t *e = &s->idemp[i];

        if (e->state != IDEMP_PENDING || !e->since_ms) {
            continue;
        }
        if (s->takeoff_pending && e->req_id == s->takeoff_req) {
            continue;
        }
        if (t - e->since_ms >= ATLINK_CMD_TOTAL_TIMEOUT_MS) {
            stale_req[n] = e->req_id;
            stale_cmd[n] = e->cmd_id;
            n++;
        }
    }
    for (i = 0; i < n; i++) {
        send_ack(s, stale_cmd[i], stale_req[i], ATLINK_RESULT_FAILED, "TIMEOUT");
    }
}

static void on_takeoff(atlink_session_t *s, const atlink_frame_t *f)
{
    atlink_takeoff_t cmd;
    atlink_on_takeoff_fn fn;
    void *ud;
    uint8_t phase = s->status.flight_phase;

    if (atlink_unpack_takeoff(f->payload, f->len, &cmd) != 0) {
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_INVALID_PARAM, "LEN");
        return;
    }
    if (!atlink_pos_ok(cmd.target_lon_e7, cmd.target_lat_e7, cmd.target_alt_mm) ||
        !atlink_pos_ok(cmd.return_lon_e7, cmd.return_lat_e7, cmd.return_alt_mm)) {
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_INVALID_PARAM, NULL);
        return;
    }
    if (phase != ATLINK_PHASE_STANDBY) {
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_DENIED, "NOT_STANDBY");
        return;
    }
    if (!s->status.gps_valid || s->status.fault_code == ATLINK_FAULT_GPS_NO_FIX) {
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_DENIED, "GPS_NO_FIX");
        return;
    }
    if (s->status.fault_code != ATLINK_FAULT_NONE) {
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_DENIED, "FC_NOT_READY");
        return;
    }
    if (!s->hooks.on_takeoff) {
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_FAILED, "NO_FC");
        return;
    }
    if (!rate_ok(s, ATLINK_CMD_TAKEOFF)) {
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_TEMPORARILY_REJECTED, "RATE");
        return;
    }

    cmd.req_id = f->req_id;
    s->mission = cmd;
    s->have_mission = 1;
    rate_stamp(s, ATLINK_CMD_TAKEOFF);
    mark_pending(s, f->req_id, f->cmd_id);
    s->takeoff_pending = 1;
    s->takeoff_req = f->req_id;
    s->takeoff_since_ms = s->now_ms;
    send_ack_transient(s, f->cmd_id, f->req_id, ATLINK_RESULT_IN_PROGRESS, NULL);

    fn = s->hooks.on_takeoff;
    ud = s->hooks.user;
    if (fn) {
        fn(&cmd, ud);
    }
}

static void on_target(atlink_session_t *s, const atlink_frame_t *f)
{
    atlink_target_fix_t fix;
    atlink_on_target_fn fn;
    void *ud;
    uint8_t phase = s->status.flight_phase;

    if (atlink_unpack_target(f->payload, f->len, &fix) != 0) {
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_INVALID_PARAM, "LEN");
        return;
    }
    if (!atlink_pos_ok(fix.lon_e7, fix.lat_e7, fix.alt_amsl_mm)) {
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_INVALID_PARAM, NULL);
        return;
    }
    if (!phase_intercept(phase)) {
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_DENIED, "NOT_INTERCEPTING");
        return;
    }
    if (!s->have_mission || fix.target_id != s->mission.target_id) {
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_DENIED, "TARGET_MISMATCH");
        return;
    }

    fix.req_id = f->req_id;
    s->mission.target_lon_e7 = fix.lon_e7;
    s->mission.target_lat_e7 = fix.lat_e7;
    s->mission.target_alt_mm = fix.alt_amsl_mm;
    send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_ACCEPTED, NULL);
    fn = s->hooks.on_target;
    ud = s->hooks.user;
    if (fn) {
        fn(&fix, ud);
    }
}

static void on_abort(atlink_session_t *s, const atlink_frame_t *f)
{
    atlink_on_abort_fn fn;
    void *ud;
    uint8_t phase = s->status.flight_phase;
    int airborne = phase_intercept(phase);

    if (phase == ATLINK_PHASE_RETURNING || phase == ATLINK_PHASE_LANDING ||
        phase == ATLINK_PHASE_LANDED) {
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_ACCEPTED, NULL);
        return;
    }
    if (s->takeoff_pending) {
        if (airborne) {
            finish_takeoff(s, ATLINK_RESULT_ACCEPTED, NULL);
        } else {
            finish_takeoff(s, ATLINK_RESULT_CANCELLED, NULL);
        }
    }
    send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_ACCEPTED, NULL);
    fn = s->hooks.on_abort;
    ud = s->hooks.user;
    if (fn) {
        fn(airborne, ud);
    }
}

static void on_period(atlink_session_t *s, const atlink_frame_t *f)
{
    uint16_t ms = atlink_rd_be16(f->payload);

    if (ms < ATLINK_PERIOD_MIN_MS || ms > ATLINK_PERIOD_MAX_MS) {
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_INVALID_PARAM, "RANGE");
        return;
    }
    if (!rate_ok(s, ATLINK_CMD_PERIOD)) {
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_TEMPORARILY_REJECTED, "RATE");
        return;
    }
    s->period_ms = ms;
    rate_stamp(s, ATLINK_CMD_PERIOD);
    send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_ACCEPTED, NULL);
}

static void on_poll(atlink_session_t *s, const atlink_frame_t *f, const atlink_cmd_info_t *info)
{
    atlink_on_poll_fn fn;
    void *ud;

    s->telem_win_open = 1;
    s->telem_req_id = f->req_id;
    fn = s->hooks.on_poll;
    ud = s->hooks.user;
    if (fn) {
        fn(info, ud);
    }
    s->telem_win_open = 0;
}

static void replay(atlink_session_t *s, const idemp_t *dup)
{
    if (dup->state == IDEMP_DONE) {
        send_ack(s, dup->cmd_id, dup->req_id, dup->result, "DUPLICATE");
    } else if (dup->state == IDEMP_PENDING) {
        send_ack_transient(s, dup->cmd_id, dup->req_id, ATLINK_RESULT_IN_PROGRESS, "PENDING");
    }
}

static void handle_frame(atlink_session_t *s, const atlink_frame_t *f)
{
    int filt = filter_downlink(f, s->local_addr);
    int broadcast;
    int need;
    idemp_t *dup;
    atlink_cmd_info_t info;

    if (filt < 0) {
        s->dir_fail++;
        return;
    }
    if (filt == 0) {
        return;
    }

    s->rx_ok++;
    s->have_down = 1;
    s->last_down_ms = s->now_ms;

    broadcast = (f->addr == ATLINK_ADDR_BROADCAST);
    info.cmd_id = f->cmd_id;
    info.req_id = f->req_id;
    info.addr = f->addr;
    info.broadcast = broadcast;
    if (s->hooks.on_rx) {
        s->hooks.on_rx(&info, f->seq, f->len ? f->payload : NULL, f->len, s->hooks.user);
    }
    if (!seq_accept(&s->down_seq, f->seq, broadcast)) {
        slog(s, 1, "seq window drop seq=%u", f->seq);
        return;
    }

    if (f->req_id == 0) {
        if (broadcast) {
            return;
        }
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_INVALID_PARAM, "REQ0");
        return;
    }

    need = atlink_cmd_payload_len(f->cmd_id);
    if (need < 0) {
        if (broadcast) {
            return;
        }
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_UNSUPPORTED, NULL);
        return;
    }
    if ((int)f->len != need) {
        if (broadcast) {
            return;
        }
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_INVALID_PARAM, "LEN");
        return;
    }
    if (broadcast && !broadcast_allowed(f->cmd_id)) {
        slog(s, 1, "broadcast rejected cmd=0x%04X", f->cmd_id);
        return;
    }
    if (f->cmd_id == ATLINK_CMD_POLL) {
        on_poll(s, f, &info);
        return;
    }

    dup = idemp_find(s, f->req_id);
    if (dup) {
        replay(s, dup);
        return;
    }

    switch (f->cmd_id) {
    case ATLINK_CMD_TAKEOFF:
        on_takeoff(s, f);
        break;
    case ATLINK_CMD_TARGET:
        on_target(s, f);
        break;
    case ATLINK_CMD_ABORT:
        on_abort(s, f);
        break;
    case ATLINK_CMD_PERIOD:
        on_period(s, f);
        break;
    default:
        send_ack(s, f->cmd_id, f->req_id, ATLINK_RESULT_UNSUPPORTED, NULL);
        break;
    }
}

atlink_session_t *atlink_session_create(uint16_t local_addr)
{
    atlink_session_t *s;

    if (local_addr == ATLINK_ADDR_BROADCAST || local_addr == ATLINK_ADDR_RESERVED) {
        return NULL;
    }
    s = calloc(1, sizeof(*s));
    if (!s) {
        return NULL;
    }
    s->local_addr = local_addr;
    s->period_ms = ATLINK_PERIOD_DEFAULT_MS;
    s->status.flight_phase = ATLINK_PHASE_UNKNOWN;
    s->status.fault_code = ATLINK_FAULT_FC_LOST;
    s->status.battery_pct = ATLINK_BATT_UNKNOWN;
    s->status.flight_direction_cdeg = ATLINK_DIRECTION_UNKNOWN;
    vision_reset(&s->vision);
    atlink_decoder_init(&s->dec);
    return s;
}

void atlink_session_destroy(atlink_session_t *s)
{
    free(s);
}

void atlink_session_set_tx(atlink_session_t *s, atlink_tx_fn fn, void *user)
{
    if (!s) {
        return;
    }
    s->tx = fn;
    s->tx_user = user;
}

void atlink_session_set_hooks(atlink_session_t *s, const atlink_hooks_t *hooks)
{
    if (!s) {
        return;
    }
    if (!hooks) {
        memset(&s->hooks, 0, sizeof(s->hooks));
        return;
    }
    s->hooks = *hooks;
}

void atlink_session_input(atlink_session_t *s, const uint8_t *data, size_t n, uint64_t now_ms)
{
    atlink_frame_t f;

    if (!s || !data || n == 0) {
        return;
    }
    note_time(s, now_ms);
    if (atlink_decoder_feed(&s->dec, data, n) < 0) {
        slog(s, 1, "decoder overflow, resync");
    }
    while (atlink_decoder_next(&s->dec, &f)) {
        handle_frame(s, &f);
    }
}

void atlink_session_tick(atlink_session_t *s, uint64_t now_ms)
{
    if (!s) {
        return;
    }
    note_time(s, now_ms);
    eval_timeouts(s);
    deliver_async(s);
}

void atlink_session_set_status(atlink_session_t *s, const atlink_status_t *st)
{
    if (!s || !st) {
        return;
    }
    s->status = *st;
    if (phase_done(st->flight_phase)) {
        s->link_lost = 0;
        s->link_lost_notify = 0;
    }
    if (s->takeoff_pending && phase_intercept(st->flight_phase)) {
        finish_takeoff(s, ATLINK_RESULT_ACCEPTED, NULL);
    }
}

void atlink_session_read_status(const atlink_session_t *s, atlink_status_t *st,
                                uint16_t *period_ms, uint16_t *fault)
{
    uint16_t f;

    if (!s) {
        return;
    }
    if (st) {
        *st = s->status;
    }
    if (period_ms) {
        *period_ms = s->period_ms;
    }
    if (fault) {
        f = s->status.fault_code;
        if (s->link_lost && f == ATLINK_FAULT_NONE) {
            f = ATLINK_FAULT_LINK_LOST_RETURNING;
        }
        *fault = f;
    }
}

void atlink_session_set_vision(atlink_session_t *s, const atlink_vision_t *vis)
{
    if (!s || !vis) {
        return;
    }
    s->vision = *vis;
}

void atlink_session_read_vision(const atlink_session_t *s, atlink_vision_t *vis)
{
    if (!s || !vis) {
        return;
    }
    *vis = s->vision;
}

uint16_t atlink_session_period_ms(const atlink_session_t *s)
{
    if (!s) {
        return ATLINK_PERIOD_DEFAULT_MS;
    }
    return s->period_ms;
}

int atlink_session_takeoff_fail(atlink_session_t *s, uint8_t result, const char *note)
{
    if (!s || !s->takeoff_pending) {
        return -1;
    }
    if (result == ATLINK_RESULT_ACCEPTED || result == ATLINK_RESULT_IN_PROGRESS) {
        return -1;
    }
    return finish_takeoff(s, result, note);
}

int atlink_session_send_telem(atlink_session_t *s, const uint8_t *payload, uint16_t len)
{
    if (!s || !payload || len != ATLINK_STATUS_LEN) {
        return -1;
    }
    if (!s->telem_win_open) {
        slog(s, 1, "0x0202 outside poll window, dropped");
        return -1;
    }
    return send_frame(s, ATLINK_CMD_STATUS, s->telem_req_id, payload, len);
}

int atlink_session_send_vision(atlink_session_t *s, const uint8_t *payload, uint16_t len)
{
    if (!s || !payload || len == 0 || len > ATLINK_VISION_MAX_LEN) {
        return -1;
    }
    if (!s->telem_win_open) {
        slog(s, 1, "0x0203 outside poll window, dropped");
        return -1;
    }
    return send_frame(s, ATLINK_CMD_VISION, s->telem_req_id, payload, len);
}
