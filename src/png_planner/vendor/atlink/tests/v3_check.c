/*
 * AT-Link v3.2 自检。向量来自协议附录 A 与第 7、8 章示例。
 * 点名先回 0x0202，再回 0x0203。0x0202 示例帧的上行序号仍按文档。
 */
#include "atlink.h"

#include <stdio.h>
#include <string.h>

extern uint16_t atlink_crc16(const uint8_t *data, size_t len);

static int g_fail;

#define CHECK(cond)                                                                 \
    do {                                                                            \
        if (!(cond)) {                                                              \
            fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond);         \
            g_fail++;                                                               \
        }                                                                           \
    } while (0)

static uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

static void wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static int unhex(const char *s, uint8_t *out, size_t cap)
{
    size_t n = 0;

    while (*s) {
        unsigned v;

        while (*s == ' ' || *s == '\n' || *s == '|') {
            s++;
        }
        if (!*s) {
            break;
        }
        if (sscanf(s, "%2x", &v) != 1) {
            return -1;
        }
        if (n >= cap) {
            return -1;
        }
        out[n++] = (uint8_t)v;
        s += 2;
    }
    return (int)n;
}

static int build_down(uint8_t *out, size_t cap, uint8_t seq, uint16_t addr, uint16_t req,
                      uint16_t cmd, const uint8_t *pl, uint16_t plen)
{
    uint16_t crc;

    if (cap < (size_t)15 + plen) {
        return -1;
    }
    out[0] = 0xEB;
    out[1] = 0x90;
    out[2] = 0x03;
    out[3] = seq;
    out[4] = 0x01;
    wr16(out + 5, addr);
    wr16(out + 7, req);
    wr16(out + 9, cmd);
    wr16(out + 11, plen);
    if (plen && pl) {
        memcpy(out + 13, pl, plen);
    }
    crc = atlink_crc16(out + 2, (size_t)11 + plen);
    wr16(out + 13 + plen, crc);
    return 15 + plen;
}

typedef struct {
    atlink_t *link;
    uint8_t   buf[16][160];
    int       len[16];
    int       n;
    int       rx_n;
    int       takeoff_n;
    int       target_n;
    int       abort_n;
    int       abort_air;
    int       lost_n;
    int       timeout_n;
    int       accept_air;
    atlink_takeoff_t    last_takeoff;
    atlink_target_fix_t last_target;
} box_t;

static int on_tx(void *user, const uint8_t *frame, uint16_t len)
{
    box_t *b = user;

    if (b->n < 16 && len < sizeof(b->buf[0])) {
        memcpy(b->buf[b->n], frame, len);
        b->len[b->n] = len;
        b->n++;
    }
    return 0;
}

static void on_rx(const atlink_cmd_info_t *info, uint8_t seq, const uint8_t *payload,
                  uint16_t len, void *user)
{
    (void)info;
    (void)seq;
    (void)payload;
    (void)len;
    ((box_t *)user)->rx_n++;
}

static void on_takeoff(const atlink_takeoff_t *cmd, void *user)
{
    box_t *b = user;

    b->takeoff_n++;
    b->last_takeoff = *cmd;
    if (b->accept_air) {
        atlink_status_t st;

        memset(&st, 0, sizeof(st));
        st.flight_phase = ATLINK_PHASE_TAKING_OFF;
        st.gps_valid = 1;
        atlink_set_status(b->link, &st);
    }
}

static void on_target(const atlink_target_fix_t *fix, void *user)
{
    box_t *b = user;

    b->target_n++;
    b->last_target = *fix;
}

static void on_abort(int airborne, void *user)
{
    box_t *b = user;

    b->abort_n++;
    b->abort_air = airborne;
}

static void on_lost(void *user)
{
    ((box_t *)user)->lost_n++;
}

static void on_timeout(uint16_t req_id, void *user)
{
    (void)req_id;
    ((box_t *)user)->timeout_n++;
}

static void box_init(box_t *b, uint16_t addr)
{
    atlink_cfg_t cfg;

    memset(b, 0, sizeof(*b));
    memset(&cfg, 0, sizeof(cfg));
    cfg.local_addr = addr;
    b->link = atlink_create(&cfg);
    CHECK(b->link != NULL);
    atlink_set_sky_tx(b->link, on_tx, b);
    atlink_set_on_rx(b->link, on_rx, b);
    atlink_set_on_takeoff(b->link, on_takeoff, b);
    atlink_set_on_target(b->link, on_target, b);
    atlink_set_on_abort(b->link, on_abort, b);
    atlink_set_on_link_lost(b->link, on_lost, b);
    atlink_set_on_takeoff_timeout(b->link, on_timeout, b);
}

static void box_close(box_t *b)
{
    atlink_destroy(b->link);
    b->link = NULL;
}

static void feed(box_t *b, const uint8_t *frame, int n, uint64_t now)
{
    atlink_sky_input(b->link, frame, (size_t)n, now);
}

static void feed_hex(box_t *b, const char *hex, uint64_t now)
{
    uint8_t raw[128];
    int n = unhex(hex, raw, sizeof(raw));

    CHECK(n > 0);
    feed(b, raw, n, now);
}

static int frame_cmd(const uint8_t *f)
{
    return rd16(f + 9);
}

static int frame_req(const uint8_t *f)
{
    return rd16(f + 7);
}

static int frame_plen(const uint8_t *f)
{
    return rd16(f + 11);
}

static const uint8_t *frame_pl(const uint8_t *f)
{
    return f + 13;
}

static int count_cmd(const box_t *b, uint16_t cmd)
{
    int i;
    int n = 0;

    for (i = 0; i < b->n; i++) {
        if (frame_cmd(b->buf[i]) == cmd) {
            n++;
        }
    }
    return n;
}

static const uint8_t *find_cmd(const box_t *b, uint16_t cmd, int which)
{
    int i;
    int seen = 0;

    for (i = 0; i < b->n; i++) {
        if (frame_cmd(b->buf[i]) == cmd) {
            if (seen == which) {
                return b->buf[i];
            }
            seen++;
        }
    }
    return NULL;
}

static void expect_ack(const uint8_t *f, uint16_t cmd, uint16_t req, uint8_t result, const char *note)
{
    const uint8_t *pl;
    int nlen;

    CHECK(f != NULL);
    if (!f) {
        return;
    }
    CHECK(frame_cmd(f) == ATLINK_CMD_ACK);
    pl = frame_pl(f);
    CHECK(rd16(pl) == cmd);
    CHECK(rd16(pl + 2) == req);
    if (pl[4] != result) {
        fprintf(stderr, "  ack cmd=%u req=%u result=%u note_len=%u want_result=%u note=%s\n",
                rd16(pl), rd16(pl + 2), pl[4], pl[5], result, note ? note : "");
    }
    CHECK(pl[4] == result);
    nlen = pl[5];
    if (!note) {
        CHECK(nlen == 0);
    } else {
        CHECK(nlen == (int)strlen(note));
        CHECK(memcmp(pl + 6, note, (size_t)nlen) == 0);
    }
}

static void fill_ready(atlink_status_t *st, uint8_t phase)
{
    memset(st, 0, sizeof(*st));
    memcpy(st->serial, "AIR001", 6);
    st->flight_phase = phase;
    st->fault_code = ATLINK_FAULT_NONE;
    st->battery_pct = 82;
    st->battery_mv = 22400;
    st->ground_speed_cms = 1250;
    st->lon_e7 = 1163980000;
    st->lat_e7 = 399090000;
    st->alt_amsl_mm = 1235000;
    st->alt_rel_mm = 35000;
    st->gps_valid = 1;
    st->flight_direction_cdeg = 23200;
}

static void set_ready(box_t *b, uint8_t phase)
{
    atlink_status_t st;

    fill_ready(&st, phase);
    atlink_set_status(b->link, &st);
}

static void check_crc(void)
{
    uint8_t v1[32];
    uint8_t v2[48];
    int n1;
    int n2;

    n1 = unhex("02 07 01 00 03 00 2A 01 02 00 04 00 00 C3 50", v1, sizeof(v1));
    n2 = unhex("03 07 01 00 03 00 2A 01 01 00 1A "
               "00 07 45 60 D9 58 17 C9 95 98 00 12 51 74 "
               "45 5F B4 60 17 C8 41 C0 00 12 01 60",
               v2, sizeof(v2));
    CHECK(n1 == 15);
    CHECK(n2 == 37);
    CHECK(atlink_crc16(v1, (size_t)n1) == 0xB125);
    CHECK(atlink_crc16(v2, (size_t)n2) == 0x3C02);
}

static void check_vectors(void)
{
    box_t b;
    const uint8_t *ack;
    const uint8_t *st;
    static const char not_standby[] = "01 01 00 2A 02 0B 4E 4F 54 5F 53 54 41 4E 44 42 59";
    uint8_t expect_note[32];
    int n;

    box_init(&b, 3);
    feed_hex(&b, "EB 90 02 01 01 00 01 00 01 01 04 00 00 48 9E", 1000);
    CHECK(b.n == 0);
    CHECK(b.rx_n == 0);

    feed_hex(&b,
             "EB 90 03 07 01 00 03 00 2A 01 01 00 1A "
             "00 07 45 60 D9 58 17 C9 95 98 00 12 51 74 "
             "45 5F B4 60 17 C8 41 C0 00 12 01 60 3C 02",
             1000);
    CHECK(b.takeoff_n == 0);
    ack = find_cmd(&b, ATLINK_CMD_ACK, 0);
    CHECK(ack != NULL);
    n = unhex(not_standby, expect_note, sizeof(expect_note));
    CHECK(n > 0 && ack && frame_plen(ack) == n);
    CHECK(ack && memcmp(frame_pl(ack), expect_note, (size_t)n) == 0);
    box_close(&b);

    box_init(&b, 3);
    set_ready(&b, ATLINK_PHASE_STANDBY);
    feed_hex(&b,
             "EB 90 03 07 01 00 03 00 2A 01 01 00 1A "
             "00 07 45 60 D9 58 17 C9 95 98 00 12 51 74 "
             "45 5F B4 60 17 C8 41 C0 00 12 01 60 3C 02",
             2000);
    CHECK(b.takeoff_n == 1);
    CHECK(b.last_takeoff.target_id == 7);
    CHECK(b.last_takeoff.target_lon_e7 == 1163975000);
    CHECK(b.last_takeoff.target_lat_e7 == 399087000);
    CHECK(b.last_takeoff.target_alt_mm == 1200500);
    CHECK(b.last_takeoff.return_lon_e7 == 1163900000);
    CHECK(b.last_takeoff.return_lat_e7 == 399000000);
    CHECK(b.last_takeoff.return_alt_mm == 1180000);
    CHECK(b.last_takeoff.req_id == 42);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), ATLINK_CMD_TAKEOFF, 42, ATLINK_RESULT_IN_PROGRESS, NULL);
    box_close(&b);

    box_init(&b, 3);
    feed_hex(&b, "EB 90 03 09 01 00 03 00 2C 01 04 00 00 93 B1", 1000);
    CHECK(b.n == 2);
    CHECK(count_cmd(&b, ATLINK_CMD_ACK) == 0);
    CHECK(count_cmd(&b, ATLINK_CMD_VISION) == 1);
    st = find_cmd(&b, ATLINK_CMD_STATUS, 0);
    CHECK(st != NULL && b.len[0] == 59);
    CHECK(st && frame_req(st) == 0x2C);
    CHECK(st && frame_plen(st) == ATLINK_STATUS_LEN);
    CHECK(st && rd16(frame_pl(st) + 42) == ATLINK_DIRECTION_UNKNOWN);
    CHECK(frame_req(b.buf[1]) == 0x2C);
    box_close(&b);

    box_init(&b, 3);
    feed_hex(&b, "EB 90 03 0D 01 00 00 00 2E 01 03 00 00 27 15", 1000);
    CHECK(b.abort_n == 1);
    CHECK(b.abort_air == 0);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), ATLINK_CMD_ABORT, 0x2E, ATLINK_RESULT_ACCEPTED, NULL);
    box_close(&b);

    box_init(&b, 3);
    feed_hex(&b, "EB 90 03 0C 01 00 03 00 2D 01 05 00 02 00 C8 D0 F7", 1000);
    CHECK(atlink_get_report_period_ms(b.link) == 200);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), ATLINK_CMD_PERIOD, 0x2D, ATLINK_RESULT_ACCEPTED, NULL);
    box_close(&b);
}

static void check_status_example(void)
{
    static const struct {
        uint8_t up_seq;
        uint16_t req;
        uint16_t period;
        const char *frame;
    } examples[] = {
        {11, 44, 200,
         "EB 90 03 0B 02 00 03 00 2C 02 02 00 2C "
         "41 49 52 30 30 31 00 00 00 00 00 00 00 00 00 00 "
         "02 00 00 52 57 80 04 E2 "
         "45 60 EC E0 17 C9 A1 50 00 12 D8 38 00 00 88 B8 00 C8 5A A0 4F 9E"},
        {20, 60, 1000,
         "EB 90 03 14 02 00 03 00 3C 02 02 00 2C "
         "41 49 52 30 30 31 00 00 00 00 00 00 00 00 00 00 "
         "00 00 00 64 62 70 00 00 "
         "45 5F B4 60 17 C8 41 C0 00 12 01 60 00 00 00 00 03 E8 FF FF 79 CC"},
    };
    size_t i;

    for (i = 0; i < sizeof(examples) / sizeof(examples[0]); i++) {
        box_t b;
        atlink_status_t status;
        uint8_t frame[32];
        uint8_t raw[64];
        uint8_t period[2];
        uint8_t seq = 0;
        const uint8_t *st;
        int n;

        box_init(&b, 3);
        fill_ready(&status, ATLINK_PHASE_INTERCEPTING);
        if (i == 1) {
            status.flight_phase = ATLINK_PHASE_STANDBY;
            status.battery_pct = 100;
            status.battery_mv = 25200;
            status.ground_speed_cms = 0;
            status.lon_e7 = 1163900000;
            status.lat_e7 = 399000000;
            status.alt_amsl_mm = 1180000;
            status.alt_rel_mm = 0;
            status.flight_direction_cdeg = ATLINK_DIRECTION_UNKNOWN;
        }
        atlink_set_status(b.link, &status);
        if (examples[i].period != ATLINK_PERIOD_DEFAULT_MS) {
            wr16(period, examples[i].period);
            n = build_down(frame, sizeof(frame), seq++, 3, 45, ATLINK_CMD_PERIOD, period, 2);
            feed(&b, frame, n, 1000);
            expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), ATLINK_CMD_PERIOD, 45,
                       ATLINK_RESULT_ACCEPTED, NULL);
        }
        /* 每次点名占两个上行序号（0x0202、0x0203）。垫到示例 0x0202 的序号。 */
        {
            int sent = b.n;

            while (sent < (int)examples[i].up_seq) {
                b.n = 0;
                n = build_down(frame, sizeof(frame), seq++, 3, (uint16_t)(100 + sent),
                               ATLINK_CMD_POLL, NULL, 0);
                feed(&b, frame, n, 1001u + (unsigned)sent);
                CHECK(b.n == 2);
                sent += b.n;
            }
            CHECK(sent == (int)examples[i].up_seq);
        }
        b.n = 0;
        n = build_down(frame, sizeof(frame), seq, 3, examples[i].req, ATLINK_CMD_POLL, NULL, 0);
        feed(&b, frame, n, 1100);
        st = find_cmd(&b, ATLINK_CMD_STATUS, 0);
        n = unhex(examples[i].frame, raw, sizeof(raw));
        CHECK(n == 59);
        CHECK(b.n == 2 && b.len[0] == n);
        CHECK(st && memcmp(st, raw, (size_t)n) == 0);
        CHECK(st && atlink_crc16(st + 2, 55) == rd16(raw + 57));
        CHECK(count_cmd(&b, ATLINK_CMD_ACK) == 0);
        CHECK(count_cmd(&b, ATLINK_CMD_VISION) == 1);
        box_close(&b);
    }
}

static void check_status_direction(void)
{
    static const struct {
        int gps_valid;
        uint16_t speed;
        uint16_t direction;
        uint16_t expected;
    } cases[] = {
        {1, 1250, 0, 0},
        {1, 1250, 9000, 9000},
        {1, 1250, 18000, 18000},
        {1, 1250, 27000, 27000},
        {1, 1250, 35999, 35999},
        {1, 1250, 36000, 0},
        {1, 1250, 36001, ATLINK_DIRECTION_UNKNOWN},
        {1, 1250, 65534, ATLINK_DIRECTION_UNKNOWN},
        {1, 1250, 9000, 9000},
        {1, 1250, ATLINK_DIRECTION_UNKNOWN, ATLINK_DIRECTION_UNKNOWN},
        {0, 1250, 9000, ATLINK_DIRECTION_UNKNOWN},
        {1, 0, 9000, ATLINK_DIRECTION_UNKNOWN},
        {1, 49, 9000, ATLINK_DIRECTION_UNKNOWN},
        {1, 50, 9000, 9000},
    };
    box_t b;
    atlink_status_t status;
    uint8_t frame[32];
    size_t i;

    box_init(&b, 3);
    fill_ready(&status, ATLINK_PHASE_INTERCEPTING);
    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        const uint8_t *st;
        int n;
        int offset;

        status.gps_valid = cases[i].gps_valid;
        status.ground_speed_cms = cases[i].speed;
        status.flight_direction_cdeg = cases[i].direction;
        atlink_set_status(b.link, &status);
        b.n = 0;
        n = build_down(frame, sizeof(frame), (uint8_t)i, 3, (uint16_t)(100 + i),
                       ATLINK_CMD_POLL, NULL, 0);
        feed(&b, frame, n, 1000u + i);
        st = find_cmd(&b, ATLINK_CMD_STATUS, 0);
        CHECK(b.n == 2 && b.len[0] == 59);
        CHECK(count_cmd(&b, ATLINK_CMD_VISION) == 1);
        CHECK(st && st[2] == 0x03);
        CHECK(st && frame_req(st) == (int)(100 + i));
        CHECK(st && frame_plen(st) == 44);
        CHECK(st && rd16(frame_pl(st) + 22) == cases[i].speed);
        CHECK(st && rd16(frame_pl(st) + 40) == 1000);
        CHECK(st && rd16(frame_pl(st) + 42) == cases[i].expected);
        CHECK(st && atlink_crc16(st + 2, 55) == rd16(st + 57));
        if (!cases[i].gps_valid) {
            for (offset = 24; offset <= 36; offset += 4) {
                CHECK(st && memcmp(frame_pl(st) + offset, "\x7F\xFF\xFF\xFF", 4) == 0);
            }
        }
    }
    box_close(&b);
}

static void check_rules(void)
{
    box_t b;
    uint8_t frame[96];
    uint8_t pl[32];
    int n;
    atlink_status_t st;
    const uint8_t *ack;

    CHECK(atlink_create(NULL) == NULL);
    {
        atlink_cfg_t cfg;
        atlink_t *bad;

        memset(&cfg, 0, sizeof(cfg));
        bad = atlink_create(&cfg);
        CHECK(bad == NULL);
    }

    box_init(&b, 3);
    n = build_down(frame, sizeof(frame), 1, 0, 10, ATLINK_CMD_TAKEOFF, NULL, 0);
    /* length will be fixed below; broadcast takeoff with a valid 26 B payload */
    memset(pl, 0, sizeof(pl));
    wr16(pl, 7);
    n = build_down(frame, sizeof(frame), 1, 0, 10, ATLINK_CMD_TAKEOFF, pl, ATLINK_TAKEOFF_LEN);
    feed(&b, frame, n, 1000);
    CHECK(b.n == 0);
    CHECK(b.takeoff_n == 0);
    n = build_down(frame, sizeof(frame), 2, 0, 11, ATLINK_CMD_POLL, NULL, 0);
    feed(&b, frame, n, 1000);
    CHECK(b.n == 0);
    box_close(&b);

    box_init(&b, 3);
    set_ready(&b, ATLINK_PHASE_STANDBY);
    b.accept_air = 1;
    memset(pl, 0, sizeof(pl));
    wr16(pl, 7);
    /* lon/lat/alt 0 are inside the legal range */
    n = build_down(frame, sizeof(frame), 1, 3, 20, ATLINK_CMD_TAKEOFF, pl, ATLINK_TAKEOFF_LEN);
    feed(&b, frame, n, 2000);
    CHECK(b.takeoff_n == 1);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), ATLINK_CMD_TAKEOFF, 20, ATLINK_RESULT_IN_PROGRESS, NULL);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 1), ATLINK_CMD_TAKEOFF, 20, ATLINK_RESULT_ACCEPTED, NULL);
    box_close(&b);

    box_init(&b, 3);
    set_ready(&b, ATLINK_PHASE_STANDBY);
    memset(pl, 0, sizeof(pl));
    wr16(pl, 7);
    n = build_down(frame, sizeof(frame), 1, 3, 21, ATLINK_CMD_TAKEOFF, pl, ATLINK_TAKEOFF_LEN);
    feed(&b, frame, n, 3000);
    b.n = 0;
    n = build_down(frame, sizeof(frame), 2, 3, 21, ATLINK_CMD_TAKEOFF, pl, ATLINK_TAKEOFF_LEN);
    feed(&b, frame, n, 3100);
    CHECK(b.takeoff_n == 1);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), ATLINK_CMD_TAKEOFF, 21, ATLINK_RESULT_IN_PROGRESS, "PENDING");
    box_close(&b);

    box_init(&b, 3);
    set_ready(&b, ATLINK_PHASE_STANDBY);
    memset(pl, 0, sizeof(pl));
    wr16(pl, 7);
    n = build_down(frame, sizeof(frame), 1, 3, 22, ATLINK_CMD_TAKEOFF, pl, ATLINK_TAKEOFF_LEN);
    feed(&b, frame, n, 1000);
    atlink_tick(b.link, 1000);
    atlink_tick(b.link, 11000);
    CHECK(b.timeout_n == 1);
    ack = find_cmd(&b, ATLINK_CMD_ACK, 1);
    expect_ack(ack, ATLINK_CMD_TAKEOFF, 22, ATLINK_RESULT_FAILED, "TIMEOUT");
    box_close(&b);

    box_init(&b, 3);
    memset(&st, 0, sizeof(st));
    st.flight_phase = ATLINK_PHASE_STANDBY;
    st.gps_valid = 0;
    atlink_set_status(b.link, &st);
    memset(pl, 0, sizeof(pl));
    wr16(pl, 1);
    n = build_down(frame, sizeof(frame), 1, 3, 30, ATLINK_CMD_TAKEOFF, pl, ATLINK_TAKEOFF_LEN);
    feed(&b, frame, n, 1000);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), ATLINK_CMD_TAKEOFF, 30, ATLINK_RESULT_DENIED, "GPS_NO_FIX");
    st.gps_valid = 1;
    st.fault_code = ATLINK_FAULT_FC_LOST;
    atlink_set_status(b.link, &st);
    b.n = 0;
    n = build_down(frame, sizeof(frame), 2, 3, 31, ATLINK_CMD_TAKEOFF, pl, ATLINK_TAKEOFF_LEN);
    feed(&b, frame, n, 1100);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), ATLINK_CMD_TAKEOFF, 31, ATLINK_RESULT_DENIED, "FC_NOT_READY");
    CHECK(b.takeoff_n == 0);
    set_ready(&b, ATLINK_PHASE_STANDBY);
    b.n = 0;
    n = build_down(frame, sizeof(frame), 3, 3, 32, ATLINK_CMD_TAKEOFF, pl, ATLINK_TAKEOFF_LEN);
    feed(&b, frame, n, 1200);
    CHECK(b.takeoff_n == 1);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), ATLINK_CMD_TAKEOFF, 32, ATLINK_RESULT_IN_PROGRESS, NULL);
    box_close(&b);

    box_init(&b, 3);
    set_ready(&b, ATLINK_PHASE_STANDBY);
    b.accept_air = 1;
    memset(pl, 0, sizeof(pl));
    wr16(pl, 7);
    n = build_down(frame, sizeof(frame), 1, 3, 40, ATLINK_CMD_TAKEOFF, pl, ATLINK_TAKEOFF_LEN);
    feed(&b, frame, n, 4000);
    /* sample target, id 7 */
    feed_hex(&b,
             "EB 90 03 08 01 00 03 00 2B 01 02 00 12 "
             "00 07 45 60 DD 40 17 C9 97 8C 00 12 51 74 6A AF 92 80 23 4F",
             4100);
    CHECK(b.target_n == 1);
    CHECK(b.last_target.target_id == 7);
    CHECK(b.last_target.lon_e7 == 1163976000);
    CHECK(b.last_target.lat_e7 == 399087500);
    CHECK(b.last_target.alt_amsl_mm == 1200500);
    CHECK(b.last_target.observed_at_s == 0x6AAF9280u);
    b.n = 0;
    wr16(pl, 8);
    n = build_down(frame, sizeof(frame), 3, 3, 41, ATLINK_CMD_TARGET, pl, ATLINK_TARGET_LEN);
    feed(&b, frame, n, 4200);
    CHECK(b.target_n == 1);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), ATLINK_CMD_TARGET, 41, ATLINK_RESULT_DENIED, "TARGET_MISMATCH");
    b.n = 0;
    wr16(pl, 7);
    n = build_down(frame, sizeof(frame), 4, 3, 42, ATLINK_CMD_TARGET, pl, ATLINK_TARGET_LEN);
    feed(&b, frame, n, 4300);
    CHECK(b.target_n == 2);
    set_ready(&b, ATLINK_PHASE_STANDBY);
    b.n = 0;
    n = build_down(frame, sizeof(frame), 5, 3, 44, ATLINK_CMD_TARGET, pl, ATLINK_TARGET_LEN);
    feed(&b, frame, n, 4400);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), ATLINK_CMD_TARGET, 44, ATLINK_RESULT_DENIED, "NOT_INTERCEPTING");
    box_close(&b);

    box_init(&b, 3);
    wr16(pl, 200);
    n = build_down(frame, sizeof(frame), 1, 3, 50, ATLINK_CMD_PERIOD, pl, 2);
    feed(&b, frame, n, 5000);
    CHECK(atlink_get_report_period_ms(b.link) == 200);
    wr16(pl, 99);
    b.n = 0;
    n = build_down(frame, sizeof(frame), 2, 3, 51, ATLINK_CMD_PERIOD, pl, 2);
    feed(&b, frame, n, 5100);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), ATLINK_CMD_PERIOD, 51, ATLINK_RESULT_INVALID_PARAM, "RANGE");
    CHECK(atlink_get_report_period_ms(b.link) == 200);
    wr16(pl, 1001);
    b.n = 0;
    n = build_down(frame, sizeof(frame), 3, 3, 52, ATLINK_CMD_PERIOD, pl, 2);
    feed(&b, frame, n, 5200);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), ATLINK_CMD_PERIOD, 52, ATLINK_RESULT_INVALID_PARAM, "RANGE");
    CHECK(atlink_get_report_period_ms(b.link) == 200);
    wr16(pl, 500);
    b.n = 0;
    n = build_down(frame, sizeof(frame), 4, 3, 53, ATLINK_CMD_PERIOD, pl, 2);
    feed(&b, frame, n, 5300);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), ATLINK_CMD_PERIOD, 53, ATLINK_RESULT_TEMPORARILY_REJECTED, "RATE");
    CHECK(atlink_get_report_period_ms(b.link) == 200);
    box_close(&b);

    box_init(&b, 3);
    set_ready(&b, ATLINK_PHASE_STANDBY);
    atlink_set_on_takeoff(b.link, NULL, NULL);
    memset(pl, 0, sizeof(pl));
    wr16(pl, 1);
    n = build_down(frame, sizeof(frame), 1, 3, 61, ATLINK_CMD_TAKEOFF, pl, ATLINK_TAKEOFF_LEN);
    feed(&b, frame, n, 1000);
    CHECK(b.takeoff_n == 0);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), ATLINK_CMD_TAKEOFF, 61, ATLINK_RESULT_FAILED, "NO_FC");
    box_close(&b);

    box_init(&b, 3);
    n = build_down(frame, sizeof(frame), 1, 3, 62, ATLINK_CMD_POLL, NULL, 0);
    feed(&b, frame, n, 1000);
    n = build_down(frame, sizeof(frame), 2, 3, 62, ATLINK_CMD_POLL, NULL, 0);
    feed(&b, frame, n, 1100);
    CHECK(count_cmd(&b, ATLINK_CMD_STATUS) == 2);
    CHECK(count_cmd(&b, ATLINK_CMD_VISION) == 2);
    CHECK(count_cmd(&b, ATLINK_CMD_ACK) == 0);
    box_close(&b);

    box_init(&b, 3);
    n = build_down(frame, sizeof(frame), 1, 3, 60, 0x0106, NULL, 0);
    feed(&b, frame, n, 1000);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), 0x0106, 60, ATLINK_RESULT_UNSUPPORTED, NULL);
    box_close(&b);

    box_init(&b, 3);
    set_ready(&b, ATLINK_PHASE_STANDBY);
    memset(pl, 0, sizeof(pl));
    wr16(pl, 7);
    n = build_down(frame, sizeof(frame), 1, 3, 70, ATLINK_CMD_TAKEOFF, pl, ATLINK_TAKEOFF_LEN);
    feed(&b, frame, n, 7000);
    b.n = 0;
    n = build_down(frame, sizeof(frame), 2, 3, 71, ATLINK_CMD_ABORT, NULL, 0);
    feed(&b, frame, n, 7100);
    CHECK(b.abort_n == 1);
    CHECK(b.abort_air == 0);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), ATLINK_CMD_TAKEOFF, 70, ATLINK_RESULT_CANCELLED, NULL);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 1), ATLINK_CMD_ABORT, 71, ATLINK_RESULT_ACCEPTED, NULL);
    box_close(&b);

    box_init(&b, 3);
    set_ready(&b, ATLINK_PHASE_INTERCEPTING);
    n = build_down(frame, sizeof(frame), 1, 3, 72, ATLINK_CMD_ABORT, NULL, 0);
    feed(&b, frame, n, 1000);
    CHECK(b.abort_n == 1);
    CHECK(b.abort_air == 1);
    set_ready(&b, ATLINK_PHASE_RETURNING);
    b.n = 0;
    b.abort_n = 0;
    n = build_down(frame, sizeof(frame), 2, 3, 73, ATLINK_CMD_ABORT, NULL, 0);
    feed(&b, frame, n, 2000);
    CHECK(b.abort_n == 0);
    expect_ack(find_cmd(&b, ATLINK_CMD_ACK, 0), ATLINK_CMD_ABORT, 73, ATLINK_RESULT_ACCEPTED, NULL);
    box_close(&b);
}

static void check_timing(void)
{
    box_t b;
    uint8_t frame[64];
    uint8_t pl[8];
    int n;
    const uint8_t *stf;

    box_init(&b, 3);
    wr16(pl, 200);
    n = build_down(frame, sizeof(frame), 1, 3, 80, ATLINK_CMD_PERIOD, pl, 2);
    feed(&b, frame, n, 1000);
    n = build_down(frame, sizeof(frame), 2, 9, 81, ATLINK_CMD_POLL, NULL, 0);
    feed(&b, frame, n, 5000);
    atlink_tick(b.link, 1000);
    atlink_tick(b.link, 11000);
    CHECK(atlink_get_report_period_ms(b.link) == 1000);

    box_close(&b);
    box_init(&b, 3);
    wr16(pl, 200);
    n = build_down(frame, sizeof(frame), 1, 3, 82, ATLINK_CMD_PERIOD, pl, 2);
    feed(&b, frame, n, 1000);
    n = build_down(frame, sizeof(frame), 50, 0, 83, ATLINK_CMD_ABORT, NULL, 0);
    feed(&b, frame, n, 9000);
    atlink_tick(b.link, 1000);
    atlink_tick(b.link, 11000);
    CHECK(atlink_get_report_period_ms(b.link) == 200);
    atlink_tick(b.link, 19000);
    CHECK(atlink_get_report_period_ms(b.link) == 1000);
    box_close(&b);

    box_init(&b, 3);
    set_ready(&b, ATLINK_PHASE_INTERCEPTING);
    atlink_tick(b.link, 0);
    atlink_tick(b.link, 10000);
    CHECK(b.lost_n == 1);
    n = build_down(frame, sizeof(frame), 1, 3, 90, ATLINK_CMD_POLL, NULL, 0);
    b.n = 0;
    feed(&b, frame, n, 10100);
    stf = find_cmd(&b, ATLINK_CMD_STATUS, 0);
    CHECK(stf != NULL);
    CHECK(stf && rd16(frame_pl(stf) + 17) == ATLINK_FAULT_LINK_LOST_RETURNING);
    atlink_tick(b.link, 20000);
    CHECK(b.lost_n == 1);
    box_close(&b);

    box_init(&b, 3);
    n = build_down(frame, sizeof(frame), 10, 3, 1, ATLINK_CMD_POLL, NULL, 0);
    feed(&b, frame, n, 1000);
    CHECK(count_cmd(&b, ATLINK_CMD_STATUS) == 1);
    n = build_down(frame, sizeof(frame), 200, 0, 2, ATLINK_CMD_ABORT, NULL, 0);
    feed(&b, frame, n, 1100);
    b.n = 0;
    n = build_down(frame, sizeof(frame), 11, 3, 3, ATLINK_CMD_POLL, NULL, 0);
    feed(&b, frame, n, 1200);
    CHECK(count_cmd(&b, ATLINK_CMD_STATUS) == 1);
    b.n = 0;
    n = build_down(frame, sizeof(frame), 51, 3, 4, ATLINK_CMD_POLL, NULL, 0);
    feed(&b, frame, n, 1300);
    CHECK(b.n == 0);
    box_close(&b);
}

static void fill_car_vision(atlink_vision_t *v)
{
    memset(v, 0, sizeof(*v));
    v->flight_phase_ext = ATLINK_PHASE_EXT_INTERCEPTING;
    v->target_valid = 1;
    v->detection_valid = 1;
    v->x1 = 100;
    v->y1 = 50;
    v->x2 = 300;
    v->y2 = 250;
    v->score_u16 = 9500;
    v->tracking = 1;
    v->track_id = 7;
    v->class_id = 1;
    v->name_len = 3;
    memcpy(v->name, "car", 3);
}

static void check_vision(void)
{
    static const char example[] =
        "EB 90 03 0C 02 00 03 00 2D 02 03 00 2C "
        "01 04 01 04 04 01 01 00 "
        "02 14 00 00 00 64 00 00 00 32 00 00 01 2C 00 00 00 FA 25 1C 01 00 "
        "03 06 00 00 00 07 00 01 "
        "04 04 03 63 61 72 "
        "09 2F";
    box_t b;
    atlink_vision_t vis;
    uint8_t frame[32];
    uint8_t period[2];
    uint8_t raw[96];
    uint8_t seq = 0;
    const uint8_t *vf;
    const uint8_t *pl;
    int n;
    int i;

    box_init(&b, 3);
    fill_car_vision(&vis);
    atlink_set_vision(b.link, &vis);
    wr16(period, 200);
    n = build_down(frame, sizeof(frame), seq++, 3, 45, ATLINK_CMD_PERIOD, period, 2);
    feed(&b, frame, n, 1000);
    /* 应答占序号 0。再点名 5 次，识别帧序号落到 12。 */
    for (i = 0; i < 5; i++) {
        b.n = 0;
        n = build_down(frame, sizeof(frame), seq++, 3, (uint16_t)(200 + i),
                       ATLINK_CMD_POLL, NULL, 0);
        feed(&b, frame, n, 1001u + (unsigned)i);
        CHECK(b.n == 2);
    }
    b.n = 0;
    n = build_down(frame, sizeof(frame), seq, 3, 0x2D, ATLINK_CMD_POLL, NULL, 0);
    feed(&b, frame, n, 1100);
    vf = find_cmd(&b, ATLINK_CMD_VISION, 0);
    n = unhex(example, raw, sizeof(raw));
    CHECK(n == 59);
    CHECK(vf && b.len[1] == n);
    CHECK(vf && memcmp(vf, raw, (size_t)n) == 0);
    CHECK(count_cmd(&b, ATLINK_CMD_ACK) == 0);
    box_close(&b);

    box_init(&b, 3);
    n = build_down(frame, sizeof(frame), 1, 3, 9, ATLINK_CMD_POLL, NULL, 0);
    feed(&b, frame, n, 1000);
    vf = find_cmd(&b, ATLINK_CMD_VISION, 0);
    CHECK(vf && frame_plen(vf) == 41);
    CHECK(vf && frame_req(vf) == 9);
    pl = frame_pl(vf);
    CHECK(pl[0] == ATLINK_VISION_VERSION && pl[1] == 4);
    CHECK(pl[4] == ATLINK_PHASE_EXT_UNKNOWN);
    CHECK(pl[5] == 0 && pl[6] == 0);
    CHECK(memcmp(pl + 10, "\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF", 16) == 0);
    CHECK(rd16(pl + 26) == 0 && pl[28] == 0);
    CHECK(memcmp(pl + 32, "\xFF\xFF\xFF\xFF", 4) == 0);
    CHECK(rd16(pl + 36) == ATLINK_CLASS_UNKNOWN);
    CHECK(pl[40] == 0);
    CHECK(vf && atlink_crc16(vf + 2, 52) == rd16(vf + 54));
    box_close(&b);

    box_init(&b, 3);
    memset(&vis, 0, sizeof(vis));
    vis.flight_phase_ext = 8;
    vis.target_valid = 2;
    vis.detection_valid = 0;
    vis.x1 = 1;
    vis.y1 = 2;
    vis.x2 = 3;
    vis.y2 = 4;
    vis.score_u16 = 9500;
    vis.tracking = 1;
    vis.track_id = ATLINK_TRACK_INVALID;
    vis.class_id = ATLINK_CLASS_UNKNOWN;
    atlink_set_vision(b.link, &vis);
    n = build_down(frame, sizeof(frame), 1, 3, 10, ATLINK_CMD_POLL, NULL, 0);
    feed(&b, frame, n, 2000);
    vf = find_cmd(&b, ATLINK_CMD_VISION, 0);
    pl = vf ? frame_pl(vf) : NULL;
    CHECK(pl && pl[4] == ATLINK_PHASE_EXT_UNKNOWN);
    CHECK(pl && pl[5] == 0 && pl[6] == 0);
    CHECK(pl && memcmp(pl + 10, "\xFF\xFF\xFF\xFF", 4) == 0);
    CHECK(pl && rd16(pl + 26) == 0 && pl[28] == 0);

    memset(&vis, 0, sizeof(vis));
    vis.flight_phase_ext = ATLINK_PHASE_EXT_INTERCEPTING;
    vis.target_valid = 1;
    vis.detection_valid = 1;
    vis.x1 = 100;
    vis.y1 = 50;
    vis.x2 = ATLINK_BBOX_NONE;
    vis.y2 = 250;
    vis.score_u16 = 10001;
    vis.tracking = 2;
    vis.name_len = 40;
    memset(vis.name, 'A', sizeof(vis.name));
    atlink_set_vision(b.link, &vis);
    b.n = 0;
    n = build_down(frame, sizeof(frame), 2, 3, 11, ATLINK_CMD_POLL, NULL, 0);
    feed(&b, frame, n, 2100);
    vf = find_cmd(&b, ATLINK_CMD_VISION, 0);
    pl = vf ? frame_pl(vf) : NULL;
    CHECK(pl && pl[6] == 0);
    CHECK(pl && memcmp(pl + 10, "\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF", 16) == 0);
    CHECK(pl && rd16(pl + 26) == 0 && pl[28] == 0);
    CHECK(vf && frame_plen(vf) == ATLINK_VISION_MAX_LEN);
    CHECK(pl && pl[40] == ATLINK_CLASS_NAME_MAX && pl[41] == 'A');

    memset(&vis, 0, sizeof(vis));
    vis.flight_phase_ext = ATLINK_PHASE_EXT_SEARCHING;
    vis.detection_valid = 1;
    vis.x1 = 1;
    vis.y1 = 2;
    vis.x2 = 3;
    vis.y2 = 4;
    vis.score_u16 = 10001;
    vis.tracking = 1;
    vis.name_len = 40;
    memset(vis.name, 'B', sizeof(vis.name));
    atlink_set_vision(b.link, &vis);
    b.n = 0;
    n = build_down(frame, sizeof(frame), 3, 3, 12, ATLINK_CMD_POLL, NULL, 0);
    feed(&b, frame, n, 2200);
    vf = find_cmd(&b, ATLINK_CMD_VISION, 0);
    pl = vf ? frame_pl(vf) : NULL;
    CHECK(vf && frame_plen(vf) == ATLINK_VISION_MAX_LEN);
    CHECK(pl && pl[4] == ATLINK_PHASE_EXT_SEARCHING && pl[6] == 1);
    CHECK(pl && rd16(pl + 26) == ATLINK_SCORE_MAX && pl[28] == 1);
    CHECK(pl && pl[39] == 33 && pl[40] == ATLINK_CLASS_NAME_MAX);
    CHECK(pl && pl[41] == 'B' && pl[72] == 'B');
    box_close(&b);
}

int main(void)
{
    check_crc();
    check_vectors();
    check_status_example();
    check_status_direction();
    check_vision();
    check_rules();
    check_timing();
    if (g_fail) {
        fprintf(stderr, "%d checks failed\n", g_fail);
        return 1;
    }
    printf("atlink v3.2 checks passed\n");
    return 0;
}
