#ifndef CORE_CRC32_H
#define CORE_CRC32_H

#include <stddef.h>
#include <stdint.h>

/* CRC-32 (IEEE 802.3, reflected, poly 0xEDB88320, init and xorout 0xFFFFFFFF). */

#define CRC32_INIT 0xFFFFFFFFu

/* Continues a running CRC; start with CRC32_INIT and finish with crc32_final(). */
uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len);

uint32_t crc32_final(uint32_t crc);

/* One-shot CRC of a buffer. */
uint32_t crc32_compute(const uint8_t *data, size_t len);

#endif /* CORE_CRC32_H */
