/*
 * 交付层：把会话接到起飞、目标、中断和点名组包。
 * 飞行动作只通过回调交给宿主。
 */
#include "atlink.h"
#include "session.h"
#include "telem.h"

#include <stdlib.h>
#include <string.h>

static atlink_session_t *sess(atlink_t *b)
{
    return b ? (atlink_session_t *)b->session : NULL;
}

static const atlink_session_t *sess_c(const atlink_t *b)
{
    return b ? (const atlink_session_t *)b->session : NULL;
}

static void stamp(atlink_t *b, uint64_t now_ms)
{
    b->now_ms = now_ms;
}

static int sky_tx_wrap(void *user, const uint8_t *frame, uint16_t len)
{
    atlink_t *b = user;

    if (!b->sky_tx) {
        return -1;
    }
    return b->sky_tx(b->sky_tx_user, frame, len);
}

static void on_log(void *user, int level, const char *msg)
{
    atlink_t *b = user;

    if (b->log) {
        b->log(b->log_user, level, msg);
    }
}

static void on_rx(const atlink_cmd_info_t *info, uint8_t seq, const uint8_t *payload,
                  uint16_t len, void *user)
{
    atlink_t *b = user;

    if (b->on_rx) {
        b->on_rx(info, seq, payload, len, b->on_rx_user);
    }
}

static void on_takeoff(const atlink_takeoff_t *cmd, void *user)
{
    atlink_t *b = user;

    if (b->on_takeoff) {
        b->on_takeoff(cmd, b->on_takeoff_user);
    }
}

static void on_target(const atlink_target_fix_t *fix, void *user)
{
    atlink_t *b = user;

    if (b->on_target) {
        b->on_target(fix, b->on_target_user);
    }
}

static void on_abort(int airborne, void *user)
{
    atlink_t *b = user;

    if (b->on_abort) {
        b->on_abort(airborne, b->on_abort_user);
    }
}

static void on_link_lost(void *user)
{
    atlink_t *b = user;

    if (b->on_link_lost) {
        b->on_link_lost(b->on_link_lost_user);
    }
}

static void on_takeoff_timeout(uint16_t req_id, void *user)
{
    atlink_t *b = user;

    if (b->on_takeoff_timeout) {
        b->on_takeoff_timeout(req_id, b->on_takeoff_timeout_user);
    }
}

static void on_poll(const atlink_cmd_info_t *info, void *user)
{
    atlink_t       *b = user;
    atlink_status_t st;
    atlink_vision_t vis;
    uint16_t        period;
    uint16_t        fault;
    uint8_t         pl[ATLINK_STATUS_LEN];
    uint8_t         vpl[ATLINK_VISION_MAX_LEN];
    int             n;

    (void)info;
    atlink_session_read_status(sess(b), &st, &period, &fault);
    n = telem_pack(&st, period, fault, pl, sizeof(pl));
    if (n > 0) {
        atlink_session_send_telem(sess(b), pl, (uint16_t)n);
    }
    atlink_session_read_vision(sess(b), &vis);
    n = vision_pack(&vis, vpl, sizeof(vpl));
    if (n > 0) {
        atlink_session_send_vision(sess(b), vpl, (uint16_t)n);
    }
}

static void bind_session(atlink_t *b)
{
    const atlink_hooks_t hooks = {
        .log                 = on_log,
        .on_rx               = on_rx,
        /* 没有宿主回调时保持 NULL，会话直接回 NO_FC，不进入 IN_PROGRESS。 */
        .on_takeoff          = b->on_takeoff ? on_takeoff : NULL,
        .on_target           = on_target,
        .on_abort            = on_abort,
        .on_link_lost        = on_link_lost,
        .on_takeoff_timeout  = on_takeoff_timeout,
        .on_poll             = on_poll,
        .user                = b,
    };

    atlink_session_set_hooks(sess(b), &hooks);
}

atlink_t *atlink_create(const atlink_cfg_t *cfg)
{
    atlink_t *b;

    if (!cfg) {
        return NULL;
    }
    b = calloc(1, sizeof(*b));
    if (!b) {
        return NULL;
    }
    b->local_addr = cfg->local_addr;
    b->session = atlink_session_create(cfg->local_addr);
    if (!b->session) {
        free(b);
        return NULL;
    }
    bind_session(b);
    return b;
}

void atlink_destroy(atlink_t *b)
{
    if (!b) {
        return;
    }
    atlink_session_destroy(sess(b));
    b->session = NULL;
    free(b);
}

void atlink_set_log(atlink_t *b, atlink_log_fn fn, void *user)
{
    if (!b) {
        return;
    }
    b->log = fn;
    b->log_user = user;
}

void atlink_set_on_rx(atlink_t *b, atlink_on_rx_fn fn, void *user)
{
    if (!b) {
        return;
    }
    b->on_rx = fn;
    b->on_rx_user = user;
}

void atlink_set_on_takeoff(atlink_t *b, atlink_on_takeoff_fn fn, void *user)
{
    if (!b) {
        return;
    }
    b->on_takeoff = fn;
    b->on_takeoff_user = user;
    bind_session(b);
}

void atlink_set_on_target(atlink_t *b, atlink_on_target_fn fn, void *user)
{
    if (!b) {
        return;
    }
    b->on_target = fn;
    b->on_target_user = user;
}

void atlink_set_on_abort(atlink_t *b, atlink_on_abort_fn fn, void *user)
{
    if (!b) {
        return;
    }
    b->on_abort = fn;
    b->on_abort_user = user;
}

void atlink_set_on_link_lost(atlink_t *b, atlink_on_link_lost_fn fn, void *user)
{
    if (!b) {
        return;
    }
    b->on_link_lost = fn;
    b->on_link_lost_user = user;
}

void atlink_set_on_takeoff_timeout(atlink_t *b, atlink_on_takeoff_timeout_fn fn, void *user)
{
    if (!b) {
        return;
    }
    b->on_takeoff_timeout = fn;
    b->on_takeoff_timeout_user = user;
}

void atlink_set_sky_tx(atlink_t *b, atlink_tx_fn fn, void *user)
{
    if (!b) {
        return;
    }
    b->sky_tx = fn;
    b->sky_tx_user = user;
    atlink_session_set_tx(sess(b), fn ? sky_tx_wrap : NULL, b);
}

void atlink_sky_input(atlink_t *b, const uint8_t *data, size_t n, uint64_t now_ms)
{
    if (!b) {
        return;
    }
    stamp(b, now_ms);
    atlink_session_input(sess(b), data, n, now_ms);
}

void atlink_tick(atlink_t *b, uint64_t now_ms)
{
    if (!b) {
        return;
    }
    stamp(b, now_ms);
    atlink_session_tick(sess(b), now_ms);
}

void atlink_set_status(atlink_t *b, const atlink_status_t *st)
{
    if (!b) {
        return;
    }
    atlink_session_set_status(sess(b), st);
}

void atlink_set_vision(atlink_t *b, const atlink_vision_t *vis)
{
    if (!b) {
        return;
    }
    atlink_session_set_vision(sess(b), vis);
}

uint16_t atlink_get_report_period_ms(const atlink_t *b)
{
    return atlink_session_period_ms(sess_c(b));
}

int atlink_takeoff_fail(atlink_t *b, uint8_t result, const char *note)
{
    if (!b) {
        return -1;
    }
    return atlink_session_takeoff_fail(sess(b), result, note);
}
