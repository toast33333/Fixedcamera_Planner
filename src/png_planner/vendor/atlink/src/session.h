#ifndef ATLINK_SESSION_H
#define ATLINK_SESSION_H

/*
 * 库内会话。对外只用 atlink_create / atlink_sky_input，不要直接调这里。
 */
#include "atlink.h"

#include <stddef.h>
#include <stdint.h>

typedef struct atlink_session atlink_session_t;

typedef void (*atlink_on_poll_fn)(const atlink_cmd_info_t *info, void *user);

typedef struct {
    atlink_log_fn                log;
    atlink_on_rx_fn              on_rx;
    atlink_on_takeoff_fn         on_takeoff;
    atlink_on_target_fn          on_target;
    atlink_on_abort_fn           on_abort;
    atlink_on_link_lost_fn       on_link_lost;
    atlink_on_takeoff_timeout_fn on_takeoff_timeout;
    atlink_on_poll_fn            on_poll;
    void                        *user;
} atlink_hooks_t;

atlink_session_t *atlink_session_create(uint16_t local_addr);
void atlink_session_destroy(atlink_session_t *s);
void atlink_session_set_tx(atlink_session_t *s, atlink_tx_fn fn, void *user);
void atlink_session_set_hooks(atlink_session_t *s, const atlink_hooks_t *hooks);
void atlink_session_input(atlink_session_t *s, const uint8_t *data, size_t n, uint64_t now_ms);
void atlink_session_tick(atlink_session_t *s, uint64_t now_ms);

void atlink_session_set_status(atlink_session_t *s, const atlink_status_t *st);
void atlink_session_read_status(const atlink_session_t *s, atlink_status_t *st,
                                uint16_t *period_ms, uint16_t *fault);
uint16_t atlink_session_period_ms(const atlink_session_t *s);
int atlink_session_takeoff_fail(atlink_session_t *s, uint8_t result, const char *note);

int atlink_session_send_telem(atlink_session_t *s, const uint8_t *payload, uint16_t len);

#endif

