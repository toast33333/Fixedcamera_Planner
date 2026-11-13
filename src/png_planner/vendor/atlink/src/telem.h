#ifndef ATLINK_TELEM_H
#define ATLINK_TELEM_H

#include "atlink.h"

#include <stddef.h>
#include <stdint.h>

/* 固定 44 字节。period_ms、fault 由会话决定，不读 st 里的同名字段。 */
int telem_pack(const atlink_status_t *st, uint16_t period_ms, uint16_t fault,
               uint8_t *out, size_t cap);

/* 0x0203 TLV。成功返回载荷长度（41–73），失败 -1。 */
int vision_pack(const atlink_vision_t *vis, uint8_t *out, size_t cap);

#endif
