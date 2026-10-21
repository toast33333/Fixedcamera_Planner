#include "frame.h"
#include "crc16.h"

#include <string.h>

uint16_t atlink_rd_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

uint32_t atlink_rd_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

int32_t atlink_rd_bei32(const uint8_t *p)
{
    return (int32_t)atlink_rd_be32(p);
}

void atlink_wr_be16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

void atlink_wr_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

void atlink_wr_bei32(uint8_t *p, int32_t v)
{
    atlink_wr_be32(p, (uint32_t)v);
}

int atlink_encode(uint8_t *out, size_t cap, const atlink_frame_t *f)
{
    if (!out || !f || f->len > ATLINK_MAX_PAYLOAD) {
        return -1;
    }
    size_t total = (size_t)ATLINK_HEADER_LEN + f->len + ATLINK_CRC_LEN;
    if (cap < total) {
        return -1;
    }
    out[0] = ATLINK_SOF0;
    out[1] = ATLINK_SOF1;
    out[2] = f->version ? f->version : ATLINK_VERSION;
    out[3] = f->seq;
    out[4] = f->dir;
    atlink_wr_be16(out + 5, f->addr);
    atlink_wr_be16(out + 7, f->req_id);
    atlink_wr_be16(out + 9, f->cmd_id);
    atlink_wr_be16(out + 11, f->len);
    if (f->len) {
        memcpy(out + ATLINK_HEADER_LEN, f->payload, f->len);
    }
    /* CRC 从版本字节算到载荷末，不含帧头 */
    uint16_t crc = atlink_crc16(out + 2, (size_t)11 + f->len);
    atlink_wr_be16(out + ATLINK_HEADER_LEN + f->len, crc);
    return (int)total;
}

void atlink_decoder_init(atlink_decoder_t *d)
{
    memset(d, 0, sizeof(*d));
}

int atlink_decoder_feed(atlink_decoder_t *d, const uint8_t *data, size_t n)
{
    if (!d || !data || n == 0) {
        return 0;
    }
    size_t room = sizeof(d->buf) - d->len;
    if (n > room) {
        d->overflow++;
        d->len = 0;
        if (n > sizeof(d->buf)) {
            n = sizeof(d->buf);
        }
        memcpy(d->buf, data, n);
        d->len = n;
        return -1;
    }
    memcpy(d->buf + d->len, data, n);
    d->len += n;
    return 0;
}

static void decoder_discard(atlink_decoder_t *d, size_t n)
{
    if (n >= d->len) {
        d->len = 0;
        return;
    }
    memmove(d->buf, d->buf + n, d->len - n);
    d->len -= n;
}

int atlink_decoder_next(atlink_decoder_t *d, atlink_frame_t *out)
{
    if (!d || !out) {
        return 0;
    }
    for (;;) {
        if (d->len < 2) {
            return 0;
        }
        size_t i = 0;
        while (i + 1 < d->len) {
            if (d->buf[i] == ATLINK_SOF0 && d->buf[i + 1] == ATLINK_SOF1) {
                break;
            }
            i++;
        }
        if (i + 1 >= d->len) {
            uint8_t last = d->buf[d->len - 1];
            d->len = 0;
            if (last == ATLINK_SOF0) {
                d->buf[0] = ATLINK_SOF0;
                d->len = 1;
            }
            return 0;
        }
        if (i > 0) {
            decoder_discard(d, i);
        }
        if (d->len < ATLINK_HEADER_LEN) {
            return 0;
        }
        uint16_t plen = atlink_rd_be16(d->buf + 11);
        if (plen > ATLINK_MAX_PAYLOAD) {
            d->len_fail++;
            decoder_discard(d, 2);
            continue;
        }
        size_t flen = (size_t)ATLINK_HEADER_LEN + plen + ATLINK_CRC_LEN;
        if (d->len < flen) {
            return 0;
        }
        uint16_t got = atlink_rd_be16(d->buf + ATLINK_HEADER_LEN + plen);
        uint16_t exp = atlink_crc16(d->buf + 2, (size_t)11 + plen);
        if (got != exp) {
            d->crc_fail++;
            decoder_discard(d, 1);
            continue;
        }
        uint8_t ver = d->buf[2];
        if (ver != ATLINK_VERSION) {
            d->ver_fail++;
            decoder_discard(d, flen);
            continue;
        }
        memset(out, 0, sizeof(*out));
        out->version = ver;
        out->seq = d->buf[3];
        out->dir = d->buf[4];
        out->addr = atlink_rd_be16(d->buf + 5);
        out->req_id = atlink_rd_be16(d->buf + 7);
        out->cmd_id = atlink_rd_be16(d->buf + 9);
        out->len = plen;
        if (plen) {
            memcpy(out->payload, d->buf + ATLINK_HEADER_LEN, plen);
        }
        decoder_discard(d, flen);
        return 1;
    }
}

