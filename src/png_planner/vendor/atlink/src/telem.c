/*
 * 0x0202 固定 42 字节。无 GPS 时四个位置写 ATLINK_POS_UNKNOWN。
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
    return ATLINK_STATUS_LEN;
}

