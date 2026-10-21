#ifndef ATLINK_TYPES_H
#define ATLINK_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* AT-Link v3。版本字节 0x03，与 v2 不兼容。 */

#define ATLINK_POS_UNKNOWN       0x7FFFFFFF
#define ATLINK_BATT_UNKNOWN      255

#define ATLINK_ADDR_BROADCAST 0x0000u
#define ATLINK_ADDR_RESERVED  0xFFFFu

#define ATLINK_CMD_TAKEOFF 0x0101u /* 起飞：目标 + 返航点，26 B，禁止广播 */
#define ATLINK_CMD_TARGET  0x0102u /* 目标实时位置，18 B，禁止广播 */
#define ATLINK_CMD_ABORT   0x0103u /* 中断返航，0 B，可广播 */
#define ATLINK_CMD_POLL    0x0104u /* 点名，0 B，只回 0x0202 */
#define ATLINK_CMD_PERIOD  0x0105u /* 上报周期，2 B，可广播 */
#define ATLINK_CMD_ACK     0x0201u /* 命令应答 */
#define ATLINK_CMD_STATUS  0x0202u /* 状态，固定 42 B，仅被点名时发 */

#define ATLINK_STATUS_LEN 42
#define ATLINK_TAKEOFF_LEN 26
#define ATLINK_TARGET_LEN 18
#define ATLINK_PERIOD_LEN 2
#define ATLINK_ACK_NOTE_MAX 64

#define ATLINK_MAX_PAYLOAD 512

#define ATLINK_PERIOD_DEFAULT_MS 1000u
#define ATLINK_PERIOD_MIN_MS     100u
#define ATLINK_PERIOD_MAX_MS     1000u
/* 连续 10 s 没有发给本机的下行：周期回到 1000，已升空则失联返航 */
#define ATLINK_LINK_LOST_MS 10000u
/* 起飞进入 IN_PROGRESS 后，10 s 内未离地则 FAILED */
#define ATLINK_TAKEOFF_TIMEOUT_MS 10000u
/* 非起飞在途命令的看门狗。起飞用 ATLINK_TAKEOFF_TIMEOUT_MS */
#define ATLINK_CMD_TOTAL_TIMEOUT_MS 2000u
/* 起飞、设置周期的最短受理间隔。点名、目标更新、中断返航不限 */
#define ATLINK_CMD_RATE_MS 1000u

#define ATLINK_PHASE_STANDBY      0
#define ATLINK_PHASE_TAKING_OFF   1
#define ATLINK_PHASE_INTERCEPTING 2
#define ATLINK_PHASE_RETURNING    3
#define ATLINK_PHASE_LANDING      4
#define ATLINK_PHASE_LANDED       5
#define ATLINK_PHASE_UNKNOWN      255

#define ATLINK_FAULT_NONE                 0
#define ATLINK_FAULT_FC_LOST              1
#define ATLINK_FAULT_GPS_NO_FIX           2
#define ATLINK_FAULT_BATTERY_LOW          3
#define ATLINK_FAULT_BATTERY_CRITICAL     4
#define ATLINK_FAULT_LINK_LOST_RETURNING  5
#define ATLINK_FAULT_INTERNAL             6

/* 结果码沿用 MAV_RESULT 的数值 */
#define ATLINK_RESULT_ACCEPTED             0
#define ATLINK_RESULT_TEMPORARILY_REJECTED 1
#define ATLINK_RESULT_DENIED               2
#define ATLINK_RESULT_UNSUPPORTED          3
#define ATLINK_RESULT_FAILED               4
#define ATLINK_RESULT_IN_PROGRESS          5
#define ATLINK_RESULT_CANCELLED            6
#define ATLINK_RESULT_INVALID_PARAM        253

/* 0x0101。req_id 由库填，载荷里没有这一字段。 */
typedef struct {
    uint16_t target_id;
    int32_t  target_lon_e7;
    int32_t  target_lat_e7;
    int32_t  target_alt_mm;
    int32_t  return_lon_e7;
    int32_t  return_lat_e7;
    int32_t  return_alt_mm;
    uint16_t req_id;
} atlink_takeoff_t;

/* 0x0102。observed_at_s 只转交，库不拿它做新鲜度判断。 */
typedef struct {
    uint16_t target_id;
    int32_t  lon_e7;
    int32_t  lat_e7;
    int32_t  alt_amsl_mm;
    uint32_t observed_at_s;
    uint16_t req_id;
} atlink_target_fix_t;

/*
 * 宿主注入的本机状态。reportPeriodMs 不在这里，由库按 0x0105 填写。
 * gps_valid 为 0 时，四个位置在 0x0202 里写成 ATLINK_POS_UNKNOWN。
 * serial 固定 16 字节，不足补 0，可以没有结尾 0。
 */
typedef struct {
    char     serial[16];
    uint8_t  flight_phase;
    uint16_t fault_code;
    uint8_t  battery_pct;
    uint16_t battery_mv;
    uint16_t ground_speed_cms;
    int32_t  lon_e7;
    int32_t  lat_e7;
    int32_t  alt_amsl_mm;
    int32_t  alt_rel_mm;
    int      gps_valid;
} atlink_status_t;

typedef struct {
    uint16_t cmd_id;
    uint16_t req_id;
    uint16_t addr;
    int      broadcast;
} atlink_cmd_info_t;

#ifdef __cplusplus
}
#endif

#endif

