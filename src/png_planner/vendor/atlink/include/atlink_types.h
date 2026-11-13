#ifndef ATLINK_TYPES_H
#define ATLINK_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* AT-Link v3.2。版本字节仍为 0x03。0x0202 仍为 44 B；点名后再发 0x0203。 */

#define ATLINK_POS_UNKNOWN       0x7FFFFFFF
#define ATLINK_BATT_UNKNOWN      255
#define ATLINK_DIRECTION_UNKNOWN 0xFFFFu
#define ATLINK_DIRECTION_MAX_CDEG 35999u
#define ATLINK_BBOX_NONE         0xFFFFFFFFu
#define ATLINK_TRACK_INVALID     (-1)
#define ATLINK_CLASS_UNKNOWN     0xFFFFu
#define ATLINK_SCORE_MAX         10000u
#define ATLINK_CLASS_NAME_MAX    32u

#define ATLINK_ADDR_BROADCAST 0x0000u
#define ATLINK_ADDR_RESERVED  0xFFFFu

#define ATLINK_CMD_TAKEOFF 0x0101u /* 起飞：目标 + 返航点，26 B，禁止广播 */
#define ATLINK_CMD_TARGET  0x0102u /* 目标实时位置，18 B，禁止广播 */
#define ATLINK_CMD_ABORT   0x0103u /* 中断返航，0 B，可广播 */
#define ATLINK_CMD_POLL    0x0104u /* 点名，0 B，先 0x0202 再 0x0203，不回 0x0201 */
#define ATLINK_CMD_PERIOD  0x0105u /* 上报周期，2 B，可广播 */
#define ATLINK_CMD_ACK     0x0201u /* 命令应答 */
#define ATLINK_CMD_STATUS  0x0202u /* 状态，固定 44 B，仅被点名时发 */
#define ATLINK_CMD_VISION  0x0203u /* 识别与飞行阶段，TLV，紧随 0x0202 */

#define ATLINK_STATUS_LEN 44
/* 四块全在、类别名 32 B 时的载荷上限。空类别名时为 41 B。 */
#define ATLINK_VISION_MAX_LEN 73
#define ATLINK_VISION_VERSION 0x01u
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

/* 0x0203 的 flight_phase_ext。与上面的 flightPhase 不是同一套数。 */
#define ATLINK_PHASE_EXT_STANDBY      0
#define ATLINK_PHASE_EXT_TAKING_OFF   1
#define ATLINK_PHASE_EXT_APPROACHING  2
#define ATLINK_PHASE_EXT_SEARCHING    3
#define ATLINK_PHASE_EXT_INTERCEPTING 4
#define ATLINK_PHASE_EXT_RETURNING    5
#define ATLINK_PHASE_EXT_LANDING      6
#define ATLINK_PHASE_EXT_LANDED       7
#define ATLINK_PHASE_EXT_UNKNOWN      255

#define ATLINK_FAULT_NONE                 0
#define ATLINK_FAULT_FC_LOST              1
#define ATLINK_FAULT_GPS_NO_FIX           2
#define ATLINK_FAULT_BATTERY_LOW          3
#define ATLINK_FAULT_BATTERY_CRITICAL     4
#define ATLINK_FAULT_LINK_LOST_RETURNING  5
#define ATLINK_FAULT_INTERNAL             6
/* Additive health alarms. Existing values 0..6 and the 0x0202 layout remain unchanged. */
#define ATLINK_FAULT_MAVROS_TIMESYNC_HIGH  7
#define ATLINK_FAULT_VISION_STALE          8
#define ATLINK_FAULT_MAVROS_STATE_STALE    9

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
    uint16_t flight_direction_cdeg;
} atlink_status_t;

typedef struct {
    uint8_t  flight_phase_ext;
    uint8_t  target_valid;
    uint8_t  detection_valid;
    uint32_t x1;
    uint32_t y1;
    uint32_t x2;
    uint32_t y2;
    uint16_t score_u16;
    uint8_t  tracking;
    int32_t  track_id;
    uint16_t class_id;
    uint8_t  name_len;
    char     name[ATLINK_CLASS_NAME_MAX];
} atlink_vision_t;

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
