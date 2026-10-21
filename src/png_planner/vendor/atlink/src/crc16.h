#ifndef ATLINK_CRC16_H
#define ATLINK_CRC16_H

#include <stddef.h>
#include <stdint.h>

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflect, xorout 0. */
uint16_t atlink_crc16(const uint8_t *data, size_t len);

#endif

