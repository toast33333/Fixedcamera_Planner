#include "frame.h"
#include "payload.h"

#include <string.h>

int atlink_cmd_payload_len(uint16_t cmd_id)
{
    switch (cmd_id) {
    case ATLINK_CMD_TAKEOFF:
        return ATLINK_TAKEOFF_LEN;
    case ATLINK_CMD_TARGET:
        return ATLINK_TARGET_LEN;
    case ATLINK_CMD_ABORT:
    case ATLINK_CMD_POLL:
        return 0;
    case ATLINK_CMD_PERIOD:
        return ATLINK_PERIOD_LEN;
    default:
        return -1;
    }
}

int atlink_pos_ok(int32_t lon_e7, int32_t lat_e7, int32_t alt_mm)
{
    if (lon_e7 < -1800000000 || lon_e7 > 1800000000) {
        return 0;
    }
    if (lat_e7 < -900000000 || lat_e7 > 900000000) {
        return 0;
    }
    if (alt_mm < -500000 || alt_mm > 20000000) {
        return 0;
    }
    return 1;
}

int atlink_unpack_takeoff(const uint8_t *payload, uint16_t len, atlink_takeoff_t *out)
{
    if (!payload || !out || len != ATLINK_TAKEOFF_LEN) {
        return -1;
    }
    memset(out, 0, sizeof(*out));
    out->target_id      = atlink_rd_be16(payload);
    out->target_lon_e7  = atlink_rd_bei32(payload + 2);
    out->target_lat_e7  = atlink_rd_bei32(payload + 6);
    out->target_alt_mm  = atlink_rd_bei32(payload + 10);
    out->return_lon_e7  = atlink_rd_bei32(payload + 14);
    out->return_lat_e7  = atlink_rd_bei32(payload + 18);
    out->return_alt_mm  = atlink_rd_bei32(payload + 22);
    return 0;
}

int atlink_unpack_target(const uint8_t *payload, uint16_t len, atlink_target_fix_t *out)
{
    if (!payload || !out || len != ATLINK_TARGET_LEN) {
        return -1;
    }
    memset(out, 0, sizeof(*out));
    out->target_id     = atlink_rd_be16(payload);
    out->lon_e7        = atlink_rd_bei32(payload + 2);
    out->lat_e7        = atlink_rd_bei32(payload + 6);
    out->alt_amsl_mm   = atlink_rd_bei32(payload + 10);
    out->observed_at_s = atlink_rd_be32(payload + 14);
    return 0;
}

int atlink_pack_ack(uint8_t *out, size_t cap, uint16_t cmd_id, uint16_t req_id,
                    uint8_t result, const char *note)
{
    size_t note_len = 0;

    if (note && note[0]) {
        note_len = strlen(note);
        if (note_len > ATLINK_ACK_NOTE_MAX) {
            note_len = ATLINK_ACK_NOTE_MAX;
        }
    }
    if (!out || cap < 6u + note_len) {
        return -1;
    }
    atlink_wr_be16(out, cmd_id);
    atlink_wr_be16(out + 2, req_id);
    out[4] = result;
    out[5] = (uint8_t)note_len;
    if (note_len) {
        memcpy(out + 6, note, note_len);
    }
    return (int)(6u + note_len);
}
