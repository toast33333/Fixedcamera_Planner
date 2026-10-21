/*
 * 地面端 AT-Link v3：切帧、命令校验、应答与 42 字节状态。
 * 不打开设备，不建线程，不指挥飞控。飞行由宿主在回调里完成。
 */

#ifndef ATLINK_H
#define ATLINK_H

#include "atlink_types.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef ATLINK_BUILD
#define ATLINK_API __attribute__((visibility("default")))
#else
#define ATLINK_API
#endif

typedef int  (*atlink_tx_fn)(void *user, const uint8_t *frame, uint16_t len);
typedef void (*atlink_log_fn)(void *user, int level, const char *msg);
typedef void (*atlink_on_rx_fn)(const atlink_cmd_info_t *info, uint8_t seq,
                                const uint8_t *payload, uint16_t len, void *user);
typedef void (*atlink_on_takeoff_fn)(const atlink_takeoff_t *cmd, void *user);
typedef void (*atlink_on_target_fn)(const atlink_target_fix_t *fix, void *user);
/* airborne：0 仍在地面（含起飞尚未离地），1 已在 TAKING_OFF / INTERCEPTING */
typedef void (*atlink_on_abort_fn)(int airborne, void *user);
typedef void (*atlink_on_link_lost_fn)(void *user);
typedef void (*atlink_on_takeoff_timeout_fn)(uint16_t req_id, void *user);

typedef struct {
    uint16_t local_addr; /* 0x0001–0xFFFE */
} atlink_cfg_t;

typedef struct atlink {
    uint16_t local_addr;
    uint64_t now_ms;

    atlink_tx_fn                 sky_tx;
    void                        *sky_tx_user;
    atlink_log_fn                log;
    void                        *log_user;
    atlink_on_rx_fn              on_rx;
    void                        *on_rx_user;
    atlink_on_takeoff_fn         on_takeoff;
    void                        *on_takeoff_user;
    atlink_on_target_fn          on_target;
    void                        *on_target_user;
    atlink_on_abort_fn           on_abort;
    void                        *on_abort_user;
    atlink_on_link_lost_fn       on_link_lost;
    void                        *on_link_lost_user;
    atlink_on_takeoff_timeout_fn on_takeoff_timeout;
    void                        *on_takeoff_timeout_user;

    void *session;
} atlink_t;

ATLINK_API atlink_t *atlink_create(const atlink_cfg_t *cfg);
ATLINK_API void      atlink_destroy(atlink_t *b);

ATLINK_API void atlink_set_log(atlink_t *b, atlink_log_fn fn, void *user);
ATLINK_API void atlink_set_on_rx(atlink_t *b, atlink_on_rx_fn fn, void *user);
ATLINK_API void atlink_set_on_takeoff(atlink_t *b, atlink_on_takeoff_fn fn, void *user);
ATLINK_API void atlink_set_on_target(atlink_t *b, atlink_on_target_fn fn, void *user);
ATLINK_API void atlink_set_on_abort(atlink_t *b, atlink_on_abort_fn fn, void *user);
ATLINK_API void atlink_set_on_link_lost(atlink_t *b, atlink_on_link_lost_fn fn, void *user);
ATLINK_API void atlink_set_on_takeoff_timeout(atlink_t *b, atlink_on_takeoff_timeout_fn fn, void *user);

ATLINK_API void atlink_set_sky_tx(atlink_t *b, atlink_tx_fn fn, void *user);
ATLINK_API void atlink_sky_input(atlink_t *b, const uint8_t *data, size_t n, uint64_t now_ms);
ATLINK_API void atlink_tick(atlink_t *b, uint64_t now_ms);

/* 整表覆盖。库在这里观察阶段，决定在途起飞是否 ACCEPTED。 */
ATLINK_API void     atlink_set_status(atlink_t *b, const atlink_status_t *st);
ATLINK_API uint16_t atlink_get_report_period_ms(const atlink_t *b);

/*
 * 在途起飞的失败终态。result 不能是 ACCEPTED 或 IN_PROGRESS。
 * 成功终态只由阶段进入 TAKING_OFF / INTERCEPTING 触发。
 * 没有在途起飞时返回 -1。
 */
ATLINK_API int atlink_takeoff_fail(atlink_t *b, uint8_t result, const char *note);

#ifdef __cplusplus
}
#endif

#endif

