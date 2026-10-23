#ifndef TESTS_MOCK_FLASH_H
#define TESTS_MOCK_FLASH_H

#include <stdbool.h>
#include <stdint.h>

#include "core/params.h"

/* NOR-like flash with two slots: erase sets 0xFF, write can only clear bits.
 * Power cut: after `budget` byte operations (one per programmed or erased byte) everything stops,
 * the interrupted operation is left partial and every later call fails until mock_flash_power_on(). */

#define MOCK_FLASH_SLOT_SIZE 512u

typedef struct {
    uint8_t mem[2][MOCK_FLASH_SLOT_SIZE];
    int32_t budget;      /* < 0: unlimited */
    bool powered;
    uint32_t ops;        /* byte operations performed so far */
    uint32_t erase_calls;
    uint32_t write_calls;
    bool fail_read_back; /* read() flips a bit to emulate a verify failure */
    params_flash_t api;
} mock_flash_t;

void mock_flash_init(mock_flash_t *m);

/* Cut power after `budget` more byte operations (< 0 disables the cut). */
void mock_flash_set_budget(mock_flash_t *m, int32_t budget);

/* Restores power and removes the cut; memory content is kept. */
void mock_flash_power_on(mock_flash_t *m);

#endif /* TESTS_MOCK_FLASH_H */
