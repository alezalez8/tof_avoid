#include <math.h>
#include <string.h>

#include "core/crc32.h"
#include "core/params.h"
#include "mock_flash.h"
#include "unity.h"

static mock_flash_t g_flash;
static params_store_t g_store;
static params_t g_p;

void setUp(void)
{
    mock_flash_init(&g_flash);
    memset(&g_store, 0, sizeof g_store);
    params_set_defaults(&g_p);
}

void tearDown(void) {}

/* ---- CRC-32 ---- */

void test_SW07_crc32_check_value(void)
{
    const uint8_t msg[] = "123456789";
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926u, crc32_compute(msg, 9u));
}

void test_SW07_crc32_incremental_equals_one_shot(void)
{
    const uint8_t msg[] = "123456789";
    uint32_t c = crc32_update(CRC32_INIT, msg, 4u);
    c = crc32_update(c, &msg[4], 5u);
    TEST_ASSERT_EQUAL_HEX32(crc32_compute(msg, 9u), crc32_final(c));
    TEST_ASSERT_EQUAL_HEX32(0u, crc32_compute(msg, 0u));
}

/* ---- Table ---- */

void test_SW07_table_has_name_type_default_min_max(void)
{
    for (int i = 0; i < (int)PARAM_COUNT; i++) {
        const param_def_t *d = params_def((param_id_t)i);
        TEST_ASSERT_NOT_NULL(d);
        TEST_ASSERT_NOT_NULL(d->name);
        TEST_ASSERT_TRUE(strlen(d->name) > 0u);
        /* Default lies within its own range. */
        TEST_ASSERT_TRUE_MESSAGE(params_in_range((param_id_t)i, d->def), d->name);
        if (d->type == PARAM_TYPE_F32) {
            TEST_ASSERT_TRUE(d->min.f <= d->max.f);
        } else {
            TEST_ASSERT_TRUE(d->min.u <= d->max.u);
        }
    }
    TEST_ASSERT_NULL(params_def(PARAM_COUNT));
}

void test_SW07_names_are_unique_and_findable(void)
{
    for (int i = 0; i < (int)PARAM_COUNT; i++) {
        const param_def_t *d = params_def((param_id_t)i);
        TEST_ASSERT_EQUAL_INT(i, (int)params_find(d->name));
    }
    TEST_ASSERT_EQUAL_INT((int)PARAM_COUNT, (int)params_find("NO_SUCH_PARAM"));
    TEST_ASSERT_EQUAL_INT((int)PARAM_COUNT, (int)params_find("SENSOR_RE"));
}

/* Start values of TZ section 15. */
void test_SW07_defaults_match_tz_section15(void)
{
    TEST_ASSERT_EQUAL_UINT32(4u, params_get_u32(&g_p, PARAM_SENSOR_RES));
    TEST_ASSERT_EQUAL_UINT32(15u, params_get_u32(&g_p, PARAM_SENSOR_FREQ_HZ));
    TEST_ASSERT_EQUAL_UINT32(PARAM_SCHEDULE_SIMULTANEOUS, params_get_u32(&g_p, PARAM_SCHEDULE));
    TEST_ASSERT_EQUAL_FLOAT(-30.0f, params_get_f32(&g_p, PARAM_MOUNT_YAW_DEG_L));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, params_get_f32(&g_p, PARAM_MOUNT_YAW_DEG_C));
    TEST_ASSERT_EQUAL_FLOAT(30.0f, params_get_f32(&g_p, PARAM_MOUNT_YAW_DEG_R));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, params_get_f32(&g_p, PARAM_MOUNT_PITCH_DEG_C));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, params_get_f32(&g_p, PARAM_MOUNT_ROLL_DEG_C));
    TEST_ASSERT_EQUAL_FLOAT(0.20f, params_get_f32(&g_p, PARAM_TOF_HEIGHT_GROUND_M));
    TEST_ASSERT_EQUAL_HEX32((1u << 5) | (1u << 9), params_get_u32(&g_p, PARAM_VALID_STATUS));
    TEST_ASSERT_EQUAL_UINT32(65535u, params_get_u32(&g_p, PARAM_SIGMA_MAX_MM));
    TEST_ASSERT_EQUAL_UINT32(0u, params_get_u32(&g_p, PARAM_SIGNAL_MIN));
    TEST_ASSERT_EQUAL_UINT32(0u, params_get_u32(&g_p, PARAM_AMBIENT_CLEAR_MAX));
    TEST_ASSERT_EQUAL_HEX32(0xFFu, params_get_u32(&g_p, PARAM_ROW_MASK));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, params_get_f32(&g_p, PARAM_CAL_TOL_DEG));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, params_get_f32(&g_p, PARAM_CAL_TOL_EL_DEG)); /* MAP-14b */
    TEST_ASSERT_EQUAL_UINT32(3u, params_get_u32(&g_p, PARAM_CLEAR_CONFIRM_FRAMES));
    TEST_ASSERT_EQUAL_UINT32(2u, params_get_u32(&g_p, PARAM_DIST_RELAX_FRAMES));
    TEST_ASSERT_EQUAL_UINT32(500u, params_get_u32(&g_p, PARAM_HOLD_MAX_MS));
    TEST_ASSERT_EQUAL_UINT32(0u, params_get_u32(&g_p, PARAM_FUSION_WINDOW_MS));
    TEST_ASSERT_EQUAL_UINT32(150u, params_get_u32(&g_p, PARAM_VALID_DATA_MAX_AGE_MS));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, params_get_f32(&g_p, PARAM_VALID_FRAC_MIN));
    TEST_ASSERT_EQUAL_UINT32(300u, params_get_u32(&g_p, PARAM_DEGRADED_TIME_MS));
    TEST_ASSERT_EQUAL_UINT32(PARAM_GROUND_KEEP, params_get_u32(&g_p, PARAM_GROUND_POLICY));
    TEST_ASSERT_EQUAL_FLOAT(0.15f, params_get_f32(&g_p, PARAM_GROUND_TOL_M));
    TEST_ASSERT_EQUAL_FLOAT(1.5f, params_get_f32(&g_p, PARAM_PHASE_H_FLIGHT_M));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, params_get_f32(&g_p, PARAM_PHASE_H_HYST_M));
    TEST_ASSERT_EQUAL_UINT32(4u, params_get_u32(&g_p, PARAM_MIN_DIST_CM));
    TEST_ASSERT_EQUAL_UINT32(300u, params_get_u32(&g_p, PARAM_MAX_DIST_CM));
    TEST_ASSERT_EQUAL_UINT32(10u, params_get_u32(&g_p, PARAM_MAV_DISTANCE_RATE_HZ));
    TEST_ASSERT_EQUAL_UINT32(115200u, params_get_u32(&g_p, PARAM_UART_BAUD));
    TEST_ASSERT_EQUAL_UINT32(0u, params_get_u32(&g_p, PARAM_MAV_SYSID));
    TEST_ASSERT_EQUAL_UINT32(196u, params_get_u32(&g_p, PARAM_MAV_COMPID));
    TEST_ASSERT_EQUAL_UINT32(PARAM_FAULT_BRAKE, params_get_u32(&g_p, PARAM_FAULT_ACTION));
    TEST_ASSERT_EQUAL_HEX32(PARAM_FAULT_BIT_SENSOR_FAILED | PARAM_FAULT_BIT_MAP_LOST |
                                PARAM_FAULT_BIT_MAP_STALE,
                            params_get_u32(&g_p, PARAM_FAULT_TRIGGER_MASK));
    TEST_ASSERT_EQUAL_UINT32(1u, params_get_u32(&g_p, PARAM_FAULT_ON_REBOOT));
    TEST_ASSERT_EQUAL_UINT32(0u, params_get_u32(&g_p, PARAM_FAULT_WALL));
    TEST_ASSERT_EQUAL_UINT32(250u, params_get_u32(&g_p, PARAM_WARN_CM));
    TEST_ASSERT_EQUAL_UINT32(120u, params_get_u32(&g_p, PARAM_CRIT_CM));
    TEST_ASSERT_EQUAL_UINT32(30u, params_get_u32(&g_p, PARAM_WARN_HYST_CM));
    TEST_ASSERT_EQUAL_UINT32(5u, params_get_u32(&g_p, PARAM_LCD_RATE_HZ));
    TEST_ASSERT_EQUAL_UINT32(0u, params_get_u32(&g_p, PARAM_NAMED_VALUE_ENABLE));
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_validate(&g_p));
}

/* ---- Range checks ---- */

void test_SW07_set_u32_checks_range(void)
{
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_set_u32(&g_p, PARAM_HOLD_MAX_MS, 5000u));
    TEST_ASSERT_EQUAL_UINT32(5000u, params_get_u32(&g_p, PARAM_HOLD_MAX_MS));
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_RANGE, params_set_u32(&g_p, PARAM_HOLD_MAX_MS, 5001u));
    TEST_ASSERT_EQUAL_UINT32(5000u, params_get_u32(&g_p, PARAM_HOLD_MAX_MS));
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_RANGE, params_set_u32(&g_p, PARAM_SENSOR_FREQ_HZ, 0u));
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_RANGE, params_set_u32(&g_p, PARAM_FAULT_ACTION, 3u));
    /* Unsigned narrow types: a value above the storage type is out of range. */
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_RANGE, params_set_u32(&g_p, PARAM_ROW_MASK, 0x1FFu));
}

void test_SW07_set_f32_checks_range_and_nan(void)
{
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_set_f32(&g_p, PARAM_CAL_TOL_DEG, 10.0f));
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_RANGE, params_set_f32(&g_p, PARAM_CAL_TOL_DEG, 10.01f));
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_RANGE, params_set_f32(&g_p, PARAM_CAL_TOL_DEG, -0.1f));
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_RANGE, params_set_f32(&g_p, PARAM_CAL_TOL_DEG, NAN));
    TEST_ASSERT_EQUAL_FLOAT(10.0f, params_get_f32(&g_p, PARAM_CAL_TOL_DEG));
}

void test_SW07_set_rejects_wrong_type_and_id(void)
{
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_TYPE, params_set_u32(&g_p, PARAM_CAL_TOL_DEG, 1u));
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_TYPE, params_set_f32(&g_p, PARAM_HOLD_MAX_MS, 1.0f));
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_ID, params_set_u32(&g_p, PARAM_COUNT, 1u));
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_ID, params_set_f32(&g_p, PARAM_COUNT, 1.0f));
    const param_value_t v = {.u = 0u};
    TEST_ASSERT_FALSE(params_in_range(PARAM_COUNT, v));
}

void test_SW07_validate_detects_out_of_range_value(void)
{
    g_p.v[PARAM_CLEAR_CONFIRM_FRAMES].u = 0u;
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_RANGE, params_validate(&g_p));
}

/* SNS-05: 4x4 or 8x8; 4x4 up to 30 Hz, 8x8 up to 15 Hz. */
void test_SW07_validate_resolution_and_frequency(void)
{
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_set_u32(&g_p, PARAM_SENSOR_RES, 6u));
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_INVALID, params_validate(&g_p));
    (void)params_set_u32(&g_p, PARAM_SENSOR_RES, 8u);
    (void)params_set_u32(&g_p, PARAM_SENSOR_FREQ_HZ, 15u);
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_validate(&g_p));
    (void)params_set_u32(&g_p, PARAM_SENSOR_FREQ_HZ, 16u);
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_INVALID, params_validate(&g_p));
    (void)params_set_u32(&g_p, PARAM_SENSOR_RES, 4u);
    (void)params_set_u32(&g_p, PARAM_SENSOR_FREQ_HZ, 30u);
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_validate(&g_p));
}

void test_SW07_validate_distance_and_warning_order(void)
{
    (void)params_set_u32(&g_p, PARAM_MIN_DIST_CM, 300u);
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_INVALID, params_validate(&g_p));
    params_set_defaults(&g_p);
    (void)params_set_u32(&g_p, PARAM_CRIT_CM, 250u);
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_INVALID, params_validate(&g_p));
}

/* ---- A/B storage ---- */

void test_SW07_both_slots_blank_load_safe_defaults_and_event(void)
{
    params_t p;
    memset(&p, 0, sizeof p);
    TEST_ASSERT_EQUAL_INT(PARAMS_LOADED_DEFAULTS, params_store_load(&g_store, &g_flash.api, &p));
    TEST_ASSERT_TRUE(g_store.defaults_event);
    TEST_ASSERT_EQUAL_UINT8(PARAMS_SLOT_NONE, g_store.active_slot);
    TEST_ASSERT_EQUAL_UINT32(PARAM_FAULT_BRAKE, params_get_u32(&p, PARAM_FAULT_ACTION));
    TEST_ASSERT_EQUAL_UINT32(PARAM_GROUND_KEEP, params_get_u32(&p, PARAM_GROUND_POLICY));
    TEST_ASSERT_EQUAL_MEMORY(&g_p, &p, sizeof p);
}

void test_SW07_first_save_goes_to_slot_a_then_alternates(void)
{
    params_t p;
    (void)params_store_load(&g_store, &g_flash.api, &p);
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_store_save(&g_store, &g_p, false));
    TEST_ASSERT_EQUAL_UINT8(PARAMS_SLOT_A, g_store.active_slot);
    TEST_ASSERT_EQUAL_UINT32(1u, g_store.active_seq);
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_store_save(&g_store, &g_p, false));
    TEST_ASSERT_EQUAL_UINT8(PARAMS_SLOT_B, g_store.active_slot);
    TEST_ASSERT_EQUAL_UINT32(2u, g_store.active_seq);
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_store_save(&g_store, &g_p, false));
    TEST_ASSERT_EQUAL_UINT8(PARAMS_SLOT_A, g_store.active_slot);
    TEST_ASSERT_EQUAL_UINT32(3u, g_store.active_seq);
}

void test_SW07_load_picks_highest_sequence(void)
{
    params_t p;
    (void)params_store_load(&g_store, &g_flash.api, &p);
    (void)params_set_u32(&g_p, PARAM_HOLD_MAX_MS, 400u);
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_store_save(&g_store, &g_p, false)); /* A, seq 1 */
    (void)params_set_u32(&g_p, PARAM_HOLD_MAX_MS, 450u);
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_store_save(&g_store, &g_p, false)); /* B, seq 2 */

    params_store_t st2;
    TEST_ASSERT_EQUAL_INT(PARAMS_LOADED_SLOT_B, params_store_load(&st2, &g_flash.api, &p));
    TEST_ASSERT_FALSE(st2.defaults_event);
    TEST_ASSERT_EQUAL_UINT32(2u, st2.active_seq);
    TEST_ASSERT_EQUAL_UINT32(450u, params_get_u32(&p, PARAM_HOLD_MAX_MS));

    (void)params_set_u32(&g_p, PARAM_HOLD_MAX_MS, 480u);
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_store_save(&st2, &g_p, false)); /* A, seq 3 */
    TEST_ASSERT_EQUAL_INT(PARAMS_LOADED_SLOT_A, params_store_load(&st2, &g_flash.api, &p));
    TEST_ASSERT_EQUAL_UINT32(480u, params_get_u32(&p, PARAM_HOLD_MAX_MS));
    TEST_ASSERT_EQUAL_UINT32(3u, st2.active_seq);
}

void test_SW07_save_refused_when_armed(void)
{
    params_t p;
    (void)params_store_load(&g_store, &g_flash.api, &p);
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_ARMED, params_store_save(&g_store, &g_p, true));
    TEST_ASSERT_EQUAL_UINT32(0u, g_flash.erase_calls);
    TEST_ASSERT_EQUAL_UINT32(0u, g_flash.write_calls);
    TEST_ASSERT_EQUAL_UINT8(PARAMS_SLOT_NONE, g_store.active_slot);
}

void test_SW07_save_refuses_invalid_set(void)
{
    params_t p;
    (void)params_store_load(&g_store, &g_flash.api, &p);
    g_p.v[PARAM_HOLD_MAX_MS].u = 9999u;
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_RANGE, params_store_save(&g_store, &g_p, false));
    TEST_ASSERT_EQUAL_UINT32(0u, g_flash.erase_calls);
}

void test_SW07_save_verify_failure_keeps_active_slot(void)
{
    params_t p;
    (void)params_store_load(&g_store, &g_flash.api, &p);
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_store_save(&g_store, &g_p, false)); /* A, seq 1 */
    g_flash.fail_read_back = true;
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_VERIFY, params_store_save(&g_store, &g_p, false));
    TEST_ASSERT_EQUAL_UINT8(PARAMS_SLOT_A, g_store.active_slot);
    TEST_ASSERT_EQUAL_UINT32(1u, g_store.active_seq);
}

void test_SW07_save_flash_error_keeps_active_slot(void)
{
    params_t p;
    (void)params_store_load(&g_store, &g_flash.api, &p);
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_store_save(&g_store, &g_p, false));
    mock_flash_set_budget(&g_flash, 0);
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_FLASH, params_store_save(&g_store, &g_p, false));
    TEST_ASSERT_EQUAL_UINT8(PARAMS_SLOT_A, g_store.active_slot);
}

void test_SW07_slot_too_small_is_rejected(void)
{
    params_flash_t small = g_flash.api;
    small.slot_size = PARAMS_IMAGE_SIZE - 1u;
    params_t p;
    TEST_ASSERT_EQUAL_INT(PARAMS_LOADED_DEFAULTS, params_store_load(&g_store, &small, &p));
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_SIZE, params_store_save(&g_store, &g_p, false));
}

void test_SW07_corrupted_active_slot_falls_back_to_other(void)
{
    params_t p;
    (void)params_store_load(&g_store, &g_flash.api, &p);
    (void)params_set_u32(&g_p, PARAM_HOLD_MAX_MS, 400u);
    (void)params_store_save(&g_store, &g_p, false); /* A, seq 1 */
    (void)params_set_u32(&g_p, PARAM_HOLD_MAX_MS, 450u);
    (void)params_store_save(&g_store, &g_p, false); /* B, seq 2 */

    g_flash.mem[PARAMS_SLOT_B][PARAMS_HEADER_SIZE + 3u] ^= 0x10u; /* one payload bit */
    TEST_ASSERT_EQUAL_INT(PARAMS_LOADED_SLOT_A, params_store_load(&g_store, &g_flash.api, &p));
    TEST_ASSERT_EQUAL_UINT32(400u, params_get_u32(&p, PARAM_HOLD_MAX_MS));
    TEST_ASSERT_EQUAL_UINT32(1u, g_store.active_seq);

    /* Next save goes to the inactive (corrupted) slot B with seq 2. */
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_store_save(&g_store, &g_p, false));
    TEST_ASSERT_EQUAL_UINT8(PARAMS_SLOT_B, g_store.active_slot);
    TEST_ASSERT_EQUAL_UINT32(2u, g_store.active_seq);
}

void test_SW07_header_fields_are_checked(void)
{
    params_t p;
    (void)params_store_load(&g_store, &g_flash.api, &p);
    (void)params_store_save(&g_store, &g_p, false); /* A, seq 1 */

    static const uint32_t offsets[] = {0u /* magic */, 4u /* format */, 6u /* length */,
                                       8u /* seq */, 12u /* schema */, 20u /* crc */};
    for (unsigned i = 0; i < sizeof offsets / sizeof offsets[0]; i++) {
        const uint8_t saved = g_flash.mem[PARAMS_SLOT_A][offsets[i]];
        g_flash.mem[PARAMS_SLOT_A][offsets[i]] ^= 0x01u;
        TEST_ASSERT_EQUAL_INT(PARAMS_LOADED_DEFAULTS,
                              params_store_load(&g_store, &g_flash.api, &p));
        g_flash.mem[PARAMS_SLOT_A][offsets[i]] = saved;
    }
    TEST_ASSERT_EQUAL_INT(PARAMS_LOADED_SLOT_A, params_store_load(&g_store, &g_flash.api, &p));
}

/* A slot with a correct CRC but a value outside [min, max] is not accepted. */
void test_SW07_slot_with_out_of_range_value_is_invalid(void)
{
    params_t p;
    (void)params_store_load(&g_store, &g_flash.api, &p);
    (void)params_store_save(&g_store, &g_p, false); /* A, seq 1 */

    uint8_t *img = g_flash.mem[PARAMS_SLOT_A];
    const uint32_t off = PARAMS_HEADER_SIZE + 4u * (uint32_t)PARAM_HOLD_MAX_MS;
    img[off] = 0xFFu;
    img[off + 1u] = 0xFFu; /* 65535 ms > max 5000 */
    uint32_t crc = crc32_update(CRC32_INIT, img, 20u);
    crc = crc32_final(crc32_update(crc, &img[PARAMS_HEADER_SIZE], 4u * (uint32_t)PARAM_COUNT));
    img[20] = (uint8_t)crc;
    img[21] = (uint8_t)(crc >> 8);
    img[22] = (uint8_t)(crc >> 16);
    img[23] = (uint8_t)(crc >> 24);
    TEST_ASSERT_EQUAL_INT(PARAMS_LOADED_DEFAULTS, params_store_load(&g_store, &g_flash.api, &p));
}

void test_SW07_read_error_makes_slot_invalid(void)
{
    params_t p;
    (void)params_store_load(&g_store, &g_flash.api, &p);
    (void)params_store_save(&g_store, &g_p, false);
    g_flash.powered = false;
    TEST_ASSERT_EQUAL_INT(PARAMS_LOADED_DEFAULTS, params_store_load(&g_store, &g_flash.api, &p));
}

/* Power cut at every possible byte of a save: after reboot the last completely written slot loads
 * (the old one if the new one is incomplete), never a mix (T-27 on host). */
void test_SW07_power_cut_at_any_point_loads_last_complete_slot(void)
{
    params_t old_p;
    params_t new_p;
    params_set_defaults(&old_p);
    params_set_defaults(&new_p);
    (void)params_set_u32(&old_p, PARAM_HOLD_MAX_MS, 400u);
    (void)params_set_u32(&new_p, PARAM_HOLD_MAX_MS, 450u);
    (void)params_set_f32(&new_p, PARAM_CAL_TOL_DEG, 3.0f);

    /* Measure the byte operations of one complete save. */
    mock_flash_init(&g_flash);
    params_t p;
    (void)params_store_load(&g_store, &g_flash.api, &p);
    (void)params_store_save(&g_store, &old_p, false);
    const uint32_t ops_before = g_flash.ops;
    (void)params_store_save(&g_store, &new_p, false);
    const uint32_t save_ops = g_flash.ops - ops_before;
    TEST_ASSERT_TRUE(save_ops >= MOCK_FLASH_SLOT_SIZE + PARAMS_IMAGE_SIZE);

    for (uint32_t cut = 0; cut <= save_ops; cut++) {
        mock_flash_init(&g_flash);
        (void)params_store_load(&g_store, &g_flash.api, &p);
        TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_store_save(&g_store, &old_p, false)); /* A */
        /* Second save into B is interrupted after `cut` byte operations. */
        mock_flash_set_budget(&g_flash, (int32_t)cut);
        const params_status_t s = params_store_save(&g_store, &new_p, false);
        mock_flash_power_on(&g_flash);

        params_store_t st;
        const params_load_result_t r = params_store_load(&st, &g_flash.api, &p);
        TEST_ASSERT_NOT_EQUAL(PARAMS_LOADED_DEFAULTS, r);
        if (r == PARAMS_LOADED_SLOT_B) {
            TEST_ASSERT_EQUAL_MEMORY(&new_p, &p, sizeof p);
            TEST_ASSERT_EQUAL_UINT32(2u, st.active_seq);
        } else {
            TEST_ASSERT_EQUAL_INT(PARAMS_LOADED_SLOT_A, r);
            TEST_ASSERT_EQUAL_MEMORY(&old_p, &p, sizeof p);
            TEST_ASSERT_EQUAL_UINT32(1u, st.active_seq);
            TEST_ASSERT_NOT_EQUAL(PARAMS_OK, s);
        }
        /* After the reboot a new save still works and wins. */
        TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_store_save(&st, &new_p, false));
        TEST_ASSERT_EQUAL_INT(r == PARAMS_LOADED_SLOT_B ? PARAMS_LOADED_SLOT_A : PARAMS_LOADED_SLOT_B,
                              params_store_load(&st, &g_flash.api, &p));
        TEST_ASSERT_EQUAL_MEMORY(&new_p, &p, sizeof p);
    }
}

/* Power cut during the very first save (no valid slot yet): defaults, never garbage. */
void test_SW07_power_cut_during_first_save_gives_defaults_or_new(void)
{
    params_t new_p;
    params_set_defaults(&new_p);
    (void)params_set_u32(&new_p, PARAM_HOLD_MAX_MS, 450u);
    for (uint32_t cut = 0; cut <= MOCK_FLASH_SLOT_SIZE + PARAMS_IMAGE_SIZE; cut += 7u) {
        params_t p;
        mock_flash_init(&g_flash);
        (void)params_store_load(&g_store, &g_flash.api, &p);
        mock_flash_set_budget(&g_flash, (int32_t)cut);
        (void)params_store_save(&g_store, &new_p, false);
        mock_flash_power_on(&g_flash);
        const params_load_result_t r = params_store_load(&g_store, &g_flash.api, &p);
        if (r == PARAMS_LOADED_DEFAULTS) {
            TEST_ASSERT_EQUAL_MEMORY(&g_p, &p, sizeof p);
        } else {
            TEST_ASSERT_EQUAL_INT(PARAMS_LOADED_SLOT_A, r);
            TEST_ASSERT_EQUAL_MEMORY(&new_p, &p, sizeof p);
        }
    }
}

void test_SW07_validate_row_mask_keeps_an_active_row(void)
{
    (void)params_set_u32(&g_p, PARAM_ROW_MASK, 0xF0u); /* rows 4..7 only: none at 4x4 */
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_INVALID, params_validate(&g_p));
    (void)params_set_u32(&g_p, PARAM_SENSOR_RES, 8u);
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_validate(&g_p));
}

/* Both slots unreadable at boot (transient): the next save must still beat the older valid slot. */
void test_SW07_save_after_boot_read_error_beats_existing_slot(void)
{
    params_t p;
    (void)params_store_load(&g_store, &g_flash.api, &p);
    (void)params_store_save(&g_store, &g_p, false); /* A, seq 1 */
    (void)params_store_save(&g_store, &g_p, false); /* B, seq 2 */
    g_flash.powered = false;
    params_store_t st;
    TEST_ASSERT_EQUAL_INT(PARAMS_LOADED_DEFAULTS, params_store_load(&st, &g_flash.api, &p));
    mock_flash_power_on(&g_flash);
    (void)params_set_u32(&g_p, PARAM_HOLD_MAX_MS, 444u);
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_store_save(&st, &g_p, false));
    TEST_ASSERT_EQUAL_UINT8(PARAMS_SLOT_A, st.active_slot);
    TEST_ASSERT_EQUAL_UINT32(3u, st.active_seq);
    TEST_ASSERT_EQUAL_INT(PARAMS_LOADED_SLOT_A, params_store_load(&st, &g_flash.api, &p));
    TEST_ASSERT_EQUAL_UINT32(444u, params_get_u32(&p, PARAM_HOLD_MAX_MS));
}

/* MAP-07a: without TOF_ALLOW_GROUND_DROP the value DROP is rejected by set and by load. */
void test_MAP07a_drop_rejected_without_build_flag(void)
{
    TEST_ASSERT_EQUAL_UINT32(0u, TOF_ALLOW_GROUND_DROP);
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_RANGE, params_set_u32(&g_p, PARAM_GROUND_POLICY, PARAM_GROUND_DROP));
    TEST_ASSERT_EQUAL_UINT32(PARAM_GROUND_KEEP, params_get_u32(&g_p, PARAM_GROUND_POLICY));

    /* A slot with DROP and a correct CRC (written by a DROP-enabled build) is not loaded. */
    params_t p;
    (void)params_store_load(&g_store, &g_flash.api, &p);
    (void)params_store_save(&g_store, &g_p, false); /* A, seq 1 */
    uint8_t *img = g_flash.mem[PARAMS_SLOT_A];
    img[PARAMS_HEADER_SIZE + 4u * (uint32_t)PARAM_GROUND_POLICY] = (uint8_t)PARAM_GROUND_DROP;
    uint32_t crc = crc32_update(CRC32_INIT, img, 20u);
    crc = crc32_final(crc32_update(crc, &img[PARAMS_HEADER_SIZE], 4u * (uint32_t)PARAM_COUNT));
    img[20] = (uint8_t)crc;
    img[21] = (uint8_t)(crc >> 8);
    img[22] = (uint8_t)(crc >> 16);
    img[23] = (uint8_t)(crc >> 24);
    TEST_ASSERT_EQUAL_INT(PARAMS_LOADED_DEFAULTS, params_store_load(&g_store, &g_flash.api, &p));
    TEST_ASSERT_EQUAL_UINT32(PARAM_GROUND_KEEP, params_get_u32(&p, PARAM_GROUND_POLICY));
}

/* SNS-12a, MAP-10a: new parameters and their defaults; format version bumped. */
void test_SNS12a_MAP10a_new_parameters(void)
{
    TEST_ASSERT_EQUAL_UINT32(0u, params_get_u32(&g_p, PARAM_ORIENT_CONFIRMED));
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_RANGE, params_set_u32(&g_p, PARAM_ORIENT_CONFIRMED, 2u));
    TEST_ASSERT_EQUAL_UINT32(0u, params_get_u32(&g_p, PARAM_AMBIENT_DEGRADED));
    TEST_ASSERT_EQUAL_INT((int)PARAM_ORIENT_CONFIRMED, (int)params_find("ORIENT_CONFIRMED"));
    TEST_ASSERT_EQUAL_INT((int)PARAM_AMBIENT_DEGRADED, (int)params_find("AMBIENT_DEGRADED"));
    TEST_ASSERT_EQUAL_UINT32(3u, PARAMS_FORMAT_VERSION);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_SW07_crc32_check_value);
    RUN_TEST(test_SW07_crc32_incremental_equals_one_shot);
    RUN_TEST(test_SW07_table_has_name_type_default_min_max);
    RUN_TEST(test_SW07_names_are_unique_and_findable);
    RUN_TEST(test_SW07_defaults_match_tz_section15);
    RUN_TEST(test_SW07_set_u32_checks_range);
    RUN_TEST(test_SW07_set_f32_checks_range_and_nan);
    RUN_TEST(test_SW07_set_rejects_wrong_type_and_id);
    RUN_TEST(test_SW07_validate_detects_out_of_range_value);
    RUN_TEST(test_SW07_validate_resolution_and_frequency);
    RUN_TEST(test_SW07_validate_distance_and_warning_order);
    RUN_TEST(test_SW07_both_slots_blank_load_safe_defaults_and_event);
    RUN_TEST(test_SW07_first_save_goes_to_slot_a_then_alternates);
    RUN_TEST(test_SW07_load_picks_highest_sequence);
    RUN_TEST(test_SW07_save_refused_when_armed);
    RUN_TEST(test_SW07_save_refuses_invalid_set);
    RUN_TEST(test_SW07_save_verify_failure_keeps_active_slot);
    RUN_TEST(test_SW07_save_flash_error_keeps_active_slot);
    RUN_TEST(test_SW07_slot_too_small_is_rejected);
    RUN_TEST(test_SW07_corrupted_active_slot_falls_back_to_other);
    RUN_TEST(test_SW07_header_fields_are_checked);
    RUN_TEST(test_SW07_slot_with_out_of_range_value_is_invalid);
    RUN_TEST(test_SW07_read_error_makes_slot_invalid);
    RUN_TEST(test_SW07_power_cut_at_any_point_loads_last_complete_slot);
    RUN_TEST(test_SW07_power_cut_during_first_save_gives_defaults_or_new);
    RUN_TEST(test_SW07_validate_row_mask_keeps_an_active_row);
    RUN_TEST(test_SW07_save_after_boot_read_error_beats_existing_slot);
    RUN_TEST(test_MAP07a_drop_rejected_without_build_flag);
    RUN_TEST(test_SNS12a_MAP10a_new_parameters);
    return UNITY_END();
}
