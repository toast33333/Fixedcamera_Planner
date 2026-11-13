/*
 * 0x0202 固定 44 字节。无 GPS 时位置与方向均写未知值。
 */
#include "telem.h"
#include "frame.h"

#include <string.h>

int telem_pack(const atlink_status_t *st, uint16_t period_ms, uint16_t fault,
               uint8_t *out, size_t cap)
{
    int32_t lon;
    int32_t lat;
    int32_t amsl;
    int32_t rel;
    uint16_t direction;

    if (!st || !out || cap < ATLINK_STATUS_LEN) {
        return -1;
    }
    memset(out, 0, ATLINK_STATUS_LEN);
    memcpy(out, st->serial, 16);
    out[16] = st->flight_phase;
    atlink_wr_be16(out + 17, fault);
    out[19] = st->battery_pct;
    atlink_wr_be16(out + 20, st->battery_mv);
    atlink_wr_be16(out + 22, st->ground_speed_cms);
    if (st->gps_valid) {
        lon  = st->lon_e7;
        lat  = st->lat_e7;
        amsl = st->alt_amsl_mm;
        rel  = st->alt_rel_mm;
    } else {
        lon = lat = amsl = rel = (int32_t)ATLINK_POS_UNKNOWN;
    }
    atlink_wr_bei32(out + 24, lon);
    atlink_wr_bei32(out + 28, lat);
    atlink_wr_bei32(out + 32, amsl);
    atlink_wr_bei32(out + 36, rel);
    atlink_wr_be16(out + 40, period_ms);
    direction = st->flight_direction_cdeg;
    if (direction == ATLINK_DIRECTION_MAX_CDEG + 1u) {
        direction = 0;
    }
    if (!st->gps_valid || st->ground_speed_cms < 50u ||
        direction > ATLINK_DIRECTION_MAX_CDEG) {
        direction = ATLINK_DIRECTION_UNKNOWN;
    }
    atlink_wr_be16(out + 42, direction);
    return ATLINK_STATUS_LEN;
}

static uint8_t vision_phase(uint8_t phase)
{
    if (phase <= ATLINK_PHASE_EXT_LANDED || phase == ATLINK_PHASE_EXT_UNKNOWN) {
        return phase;
    }
    return ATLINK_PHASE_EXT_UNKNOWN;
}

int vision_pack(const atlink_vision_t *vis, uint8_t *out, size_t cap)
{
    uint8_t phase;
    uint8_t target_valid;
    uint8_t detection_valid;
    uint8_t tracking;
    uint8_t name_len;
    uint16_t score;
    uint32_t x1;
    uint32_t y1;
    uint32_t x2;
    uint32_t y2;
    size_t need;
    uint8_t *p;

    if (!vis || !out) {
        return -1;
    }
    phase = vision_phase(vis->flight_phase_ext);
    target_valid = (vis->target_valid == 1) ? 1 : 0;
    detection_valid = (vis->detection_valid == 1) ? 1 : 0;
    name_len = vis->name_len;
    if (name_len > ATLINK_CLASS_NAME_MAX) {
        name_len = ATLINK_CLASS_NAME_MAX;
    }
    x1 = vis->x1;
    y1 = vis->y1;
    x2 = vis->x2;
    y2 = vis->y2;
    score = vis->score_u16;
    tracking = (vis->tracking == 1) ? 1 : 0;
    if (!detection_valid || x1 == ATLINK_BBOX_NONE || y1 == ATLINK_BBOX_NONE ||
        x2 == ATLINK_BBOX_NONE || y2 == ATLINK_BBOX_NONE) {
        detection_valid = 0;
        x1 = y1 = x2 = y2 = ATLINK_BBOX_NONE;
        score = 0;
        tracking = 0;
    } else if (score > ATLINK_SCORE_MAX) {
        score = ATLINK_SCORE_MAX;
    }

    /* version+count + 阶段块 + 识别框 + 目标标识 + 类别名（含 name_length） */
    need = 41u + name_len;
    if (cap < need) {
        return -1;
    }
    p = out;
    *p++ = ATLINK_VISION_VERSION;
    *p++ = 4;

    *p++ = 0x01;
    *p++ = 4;
    *p++ = phase;
    *p++ = target_valid;
    *p++ = detection_valid;
    *p++ = 0;

    *p++ = 0x02;
    *p++ = 20;
    atlink_wr_be32(p, x1);
    p += 4;
    atlink_wr_be32(p, y1);
    p += 4;
    atlink_wr_be32(p, x2);
    p += 4;
    atlink_wr_be32(p, y2);
    p += 4;
    atlink_wr_be16(p, score);
    p += 2;
    *p++ = tracking;
    *p++ = 0;

    *p++ = 0x03;
    *p++ = 6;
    atlink_wr_bei32(p, vis->track_id);
    p += 4;
    atlink_wr_be16(p, vis->class_id);
    p += 2;

    *p++ = 0x04;
    *p++ = (uint8_t)(1u + name_len);
    *p++ = name_len;
    if (name_len) {
        memcpy(p, vis->name, name_len);
    }
    return (int)need;
}
