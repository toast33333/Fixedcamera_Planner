#ifndef ATLINK_PAYLOAD_H
#define ATLINK_PAYLOAD_H

#include "atlink_types.h"

#include <stddef.h>
#include <stdint.h>

/* 已知下行命令的载荷长度。未知命令返回 -1。 */
int atlink_cmd_payload_len(uint16_t cmd_id);

/* 经度 [-180,180] 度、纬度 [-90,90] 度、海拔 [-500,20000] 米。 */
int atlink_pos_ok(int32_t lon_e7, int32_t lat_e7, int32_t alt_mm);

int atlink_unpack_takeoff(const uint8_t *payload, uint16_t len, atlink_takeoff_t *out);
int atlink_unpack_target(const uint8_t *payload, uint16_t len, atlink_target_fix_t *out);

/* 组 0x0201 载荷。说明最长 ATLINK_ACK_NOTE_MAX。成功返回长度，失败 -1。 */
int atlink_pack_ack(uint8_t *out, size_t cap, uint16_t cmd_id, uint16_t req_id,
                    uint8_t result, const char *note);

#endif

