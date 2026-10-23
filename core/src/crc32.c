#include "core/crc32.h"

/* REQ: SW-07 (bitwise: the parameter image is small, no 1 KiB table needed) */
uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8u; bit++) {
            const uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return crc;
}

/* REQ: SW-07 */
uint32_t crc32_final(uint32_t crc)
{
    return crc ^ 0xFFFFFFFFu;
}

/* REQ: SW-07 */
uint32_t crc32_compute(const uint8_t *data, size_t len)
{
    return crc32_final(crc32_update(CRC32_INIT, data, len));
}
