#include "mock_flash.h"

#include <string.h>

/* Consumes one byte operation; false once power is gone. */
static bool take_op(mock_flash_t *m)
{
    if (!m->powered) {
        return false;
    }
    if (m->budget == 0) {
        m->powered = false;
        return false;
    }
    if (m->budget > 0) {
        m->budget--;
    }
    m->ops++;
    return true;
}

static bool mock_erase(void *ctx, uint8_t slot)
{
    mock_flash_t *m = (mock_flash_t *)ctx;
    if (slot > 1u || !m->powered) {
        return false;
    }
    m->erase_calls++;
    for (uint32_t i = 0; i < MOCK_FLASH_SLOT_SIZE; i++) {
        if (!take_op(m)) {
            return false;
        }
        m->mem[slot][i] = 0xFFu;
    }
    return true;
}

static bool mock_write(void *ctx, uint8_t slot, uint32_t offset, const uint8_t *src, uint32_t len)
{
    mock_flash_t *m = (mock_flash_t *)ctx;
    if (slot > 1u || offset > MOCK_FLASH_SLOT_SIZE || len > MOCK_FLASH_SLOT_SIZE - offset ||
        !m->powered) {
        return false;
    }
    m->write_calls++;
    for (uint32_t i = 0; i < len; i++) {
        if (!take_op(m)) {
            return false;
        }
        m->mem[slot][offset + i] &= src[i];
    }
    return true;
}

static bool mock_read(void *ctx, uint8_t slot, uint32_t offset, uint8_t *dst, uint32_t len)
{
    mock_flash_t *m = (mock_flash_t *)ctx;
    if (slot > 1u || offset > MOCK_FLASH_SLOT_SIZE || len > MOCK_FLASH_SLOT_SIZE - offset ||
        !m->powered) {
        return false;
    }
    memcpy(dst, &m->mem[slot][offset], len);
    if (m->fail_read_back && len > 0u) {
        dst[0] ^= 0x01u;
    }
    return true;
}

void mock_flash_init(mock_flash_t *m)
{
    memset(m, 0, sizeof *m);
    memset(m->mem, 0xFF, sizeof m->mem);
    m->budget = -1;
    m->powered = true;
    m->api.ctx = m;
    m->api.slot_size = MOCK_FLASH_SLOT_SIZE;
    m->api.erase = mock_erase;
    m->api.write = mock_write;
    m->api.read = mock_read;
}

void mock_flash_set_budget(mock_flash_t *m, int32_t budget)
{
    m->budget = budget;
}

void mock_flash_power_on(mock_flash_t *m)
{
    m->powered = true;
    m->budget = -1;
}
