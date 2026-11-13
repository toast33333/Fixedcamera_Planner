#ifndef ATLINK_FRAME_H
#define ATLINK_FRAME_H

/*
 * 库内帧层。组帧、切帧、帧上的大端读写。不包含命令语义。
 *
 * AT-Link v3，多字节大端，总长 = 15 + N：
 *
 *  偏移  长度  字段
 *   0     2    帧头 EB 90（不参与 CRC）
 *   2     1    版本，本版 0x03
 *   3     1    序列号，按对端独立计数
 *   4     1    方向 0x01 下行 / 0x02 上行
 *   5     2    地址：下行=目标，上行=源；0x0000 广播
 *   7     2    请求标识，应答原样回填
 *   9     2    命令标识
 *  11     2    载荷长度 N，上限 512
 *  13     N    载荷
 *  13+N   2    CRC-16/CCITT-FALSE（范围：偏移 2 至载荷末）
 */

#include "atlink_types.h"

#include <stddef.h>
#include <stdint.h>

#define ATLINK_SOF0       0xEBu
#define ATLINK_SOF1       0x90u
#define ATLINK_VERSION    0x03u
#define ATLINK_DIR_DOWN   0x01u
#define ATLINK_DIR_UP     0x02u
#define ATLINK_HEADER_LEN 13
#define ATLINK_CRC_LEN    2
#define ATLINK_MAX_FRAME  (ATLINK_HEADER_LEN + ATLINK_MAX_PAYLOAD + ATLINK_CRC_LEN)

typedef struct {
    uint8_t  version;
    uint8_t  seq;
    uint8_t  dir;
    uint16_t addr;
    uint16_t req_id;
    uint16_t cmd_id;
    uint16_t len; /* 仅载荷字节数 */
    uint8_t  payload[ATLINK_MAX_PAYLOAD];
} atlink_frame_t;

typedef struct {
    uint8_t  buf[ATLINK_MAX_FRAME * 2];
    size_t   len;
    uint32_t crc_fail;
    uint32_t ver_fail;
    uint32_t len_fail;
    uint32_t overflow;
} atlink_decoder_t;

uint16_t atlink_rd_be16(const uint8_t *p);
uint32_t atlink_rd_be32(const uint8_t *p);
int32_t  atlink_rd_bei32(const uint8_t *p);
void     atlink_wr_be16(uint8_t *p, uint16_t v);
void     atlink_wr_be32(uint8_t *p, uint32_t v);
void     atlink_wr_bei32(uint8_t *p, int32_t v);

/* 组完整帧。成功返回帧长，失败 -1。 */
int atlink_encode(uint8_t *out, size_t cap, const atlink_frame_t *f);

void atlink_decoder_init(atlink_decoder_t *d);
/* 0 成功。缓冲装不下时清空重同步、拷入本次数据，返回 -1。 */
int  atlink_decoder_feed(atlink_decoder_t *d, const uint8_t *data, size_t n);
/* 取出一帧。1 成功，0 需要更多数据。CRC/版本错误在内部丢弃并计数。 */
int  atlink_decoder_next(atlink_decoder_t *d, atlink_frame_t *out);

#endif
