#include <string.h>

#include "core/cell_fsm.h"
#include "unity.h"

static cell_cfg_t g_cfg;
static cell_t g_c;

#define MS 1000u

void setUp(void)
{
    params_t p;
    params_set_defaults(&p);
    cell_cfg_from_params(&g_cfg, &p);
    g_cfg.clear_enabled = true; /* most tests need CLEAR_MEASURED; disabled case tested separately */
    cell_init(&g_c);
}

void tearDown(void) {}

static uint8_t occ(uint16_t cm, uint32_t t)
{
    cell_input_t in;
    memset(&in, 0, sizeof in);
    in.kind = CELL_CAND_OCC;
    in.dist_cm = cm;
    in.src_sensor = 1u;
    in.src_zone = (uint8_t)(cm % 16u);
    in.angle_ok = true;
    return cell_fsm_step(&g_c, &g_cfg, &in, t);
}

static uint8_t clr(uint32_t t)
{
    cell_input_t in;
    memset(&in, 0, sizeof in);
    in.kind = CELL_CAND_CLR;
    return cell_fsm_step(&g_c, &g_cfg, &in, t);
}

static uint8_t inv(uint32_t t)
{
    cell_input_t in;
    memset(&in, 0, sizeof in);
    in.kind = CELL_CAND_INV;
    return cell_fsm_step(&g_c, &g_cfg, &in, t);
}

static void make_clear(uint32_t t)
{
    for (unsigned k = 0; k < g_cfg.clear_confirm_frames; k++) {
        (void)clr(t + k * MS);
    }
    TEST_ASSERT_EQUAL_INT(CELL_CLEAR_MEASURED, g_c.state);
}

void test_MAP13_config_from_params(void)
{
    params_t p;
    params_set_defaults(&p);
    cell_cfg_t cfg;
    cell_cfg_from_params(&cfg, &p);
    TEST_ASSERT_EQUAL_UINT8(3u, cfg.clear_confirm_frames);
    TEST_ASSERT_EQUAL_UINT8(2u, cfg.dist_relax_frames);
    TEST_ASSERT_EQUAL_UINT32(500u * MS, cfg.hold_max_us);
    /* AMBIENT_CLEAR_MAX = 0 by default: CLEAR_MEASURED is off. */
    TEST_ASSERT_FALSE(cfg.clear_enabled);
    (void)params_set_u32(&p, PARAM_AMBIENT_CLEAR_MAX, 100u);
    cell_cfg_from_params(&cfg, &p);
    TEST_ASSERT_TRUE(cfg.clear_enabled);
}

void test_MAP13_initial_state_unknown(void)
{
    TEST_ASSERT_EQUAL_INT(CELL_UNKNOWN, g_c.state);
    TEST_ASSERT_FALSE(g_c.ground_suspect);
}

/* Any + OCC closer than the held distance -> OCCUPIED at once. */
void test_MAP13_unknown_occ_occupied_immediately(void)
{
    TEST_ASSERT_EQUAL_HEX8(CELL_EVT_OCCUPIED, occ(150u, 10u * MS));
    TEST_ASSERT_EQUAL_INT(CELL_OCCUPIED, g_c.state);
    TEST_ASSERT_EQUAL_UINT16(150u, g_c.dist_cm);
    TEST_ASSERT_EQUAL_UINT32(10u * MS, g_c.t_valid_us);
    TEST_ASSERT_EQUAL_UINT8(1u, g_c.src_sensor);
    TEST_ASSERT_EQUAL_UINT8(150u % 16u, g_c.src_zone);
    TEST_ASSERT_TRUE(g_c.angle_ok);
}

void test_MAP13_clear_occ_occupied_immediately(void)
{
    make_clear(0u);
    TEST_ASSERT_EQUAL_HEX8(CELL_EVT_OCCUPIED, occ(250u, 10u * MS));
    TEST_ASSERT_EQUAL_INT(CELL_OCCUPIED, g_c.state);
    TEST_ASSERT_EQUAL_UINT16(250u, g_c.dist_cm);
}

void test_MAP13_occupied_closer_occ_updates_immediately(void)
{
    (void)occ(200u, 0u);
    TEST_ASSERT_EQUAL_HEX8(CELL_EVT_NONE, occ(120u, 66u * MS));
    TEST_ASSERT_EQUAL_INT(CELL_OCCUPIED, g_c.state);
    TEST_ASSERT_EQUAL_UINT16(120u, g_c.dist_cm);
    TEST_ASSERT_EQUAL_UINT32(66u * MS, g_c.t_valid_us);
}

void test_MAP13_occupied_equal_occ_refreshes_time(void)
{
    (void)occ(200u, 0u);
    (void)occ(200u, 66u * MS);
    TEST_ASSERT_EQUAL_UINT16(200u, g_c.dist_cm);
    TEST_ASSERT_EQUAL_UINT32(66u * MS, g_c.t_valid_us);
}

/* OCCUPIED + OCC farther: the distance grows after DIST_RELAX_FRAMES (2) confirmations in a row. */
void test_MAP13_occupied_farther_occ_relaxes_after_2_frames(void)
{
    (void)occ(100u, 0u);
    (void)occ(180u, 66u * MS);
    TEST_ASSERT_EQUAL_UINT16(100u, g_c.dist_cm); /* one confirmation is not enough */
    TEST_ASSERT_EQUAL_UINT32(66u * MS, g_c.t_valid_us); /* still a valid obstacle measurement */
    (void)occ(180u, 133u * MS);
    TEST_ASSERT_EQUAL_INT(CELL_OCCUPIED, g_c.state);
    TEST_ASSERT_EQUAL_UINT16(180u, g_c.dist_cm);
}

/* The relaxed distance is the nearest of the confirming frames (conservative). */
void test_MAP13_relax_takes_nearest_confirmation(void)
{
    (void)occ(100u, 0u);
    (void)occ(170u, 66u * MS);
    (void)occ(190u, 133u * MS);
    TEST_ASSERT_EQUAL_UINT16(170u, g_c.dist_cm);
    TEST_ASSERT_EQUAL_UINT8(170u % 16u, g_c.src_zone);
}

void test_MAP13_relax_run_broken_by_inv(void)
{
    (void)occ(100u, 0u);
    (void)occ(180u, 66u * MS);
    (void)inv(100u * MS);
    (void)occ(180u, 133u * MS);
    TEST_ASSERT_EQUAL_UINT16(100u, g_c.dist_cm);
    (void)occ(180u, 200u * MS);
    TEST_ASSERT_EQUAL_UINT16(180u, g_c.dist_cm);
}

void test_MAP13_relax_run_broken_by_clr(void)
{
    (void)occ(100u, 0u);
    (void)occ(180u, 66u * MS);
    (void)clr(100u * MS);
    (void)occ(180u, 133u * MS);
    TEST_ASSERT_EQUAL_UINT16(100u, g_c.dist_cm);
}

void test_MAP13_relax_run_broken_by_closer_occ(void)
{
    (void)occ(100u, 0u);
    (void)occ(180u, 66u * MS);
    (void)occ(90u, 100u * MS);
    (void)occ(180u, 133u * MS);
    TEST_ASSERT_EQUAL_UINT16(90u, g_c.dist_cm);
}

/* OCCUPIED + CLR: after CLEAR_CONFIRM_FRAMES (3) in a row -> CLEAR_MEASURED (if enabled). */
void test_MAP13_occupied_clr_3_frames_clear_measured(void)
{
    (void)occ(100u, 0u);
    TEST_ASSERT_EQUAL_HEX8(CELL_EVT_NONE, clr(66u * MS));
    TEST_ASSERT_EQUAL_HEX8(CELL_EVT_NONE, clr(133u * MS));
    TEST_ASSERT_EQUAL_INT(CELL_OCCUPIED, g_c.state);
    TEST_ASSERT_EQUAL_HEX8(CELL_EVT_RELEASED, clr(200u * MS));
    TEST_ASSERT_EQUAL_INT(CELL_CLEAR_MEASURED, g_c.state);
}

/* ... otherwise UNKNOWN. */
void test_MAP13_occupied_clr_3_frames_unknown_when_clear_disabled(void)
{
    g_cfg.clear_enabled = false;
    (void)occ(100u, 0u);
    (void)clr(66u * MS);
    (void)clr(133u * MS);
    TEST_ASSERT_EQUAL_INT(CELL_OCCUPIED, g_c.state);
    TEST_ASSERT_EQUAL_HEX8(CELL_EVT_RELEASED, clr(200u * MS));
    TEST_ASSERT_EQUAL_INT(CELL_UNKNOWN, g_c.state);
}

/* OCCUPIED + INV: hold. Invalid data never removes an obstacle. */
void test_MAP13_occupied_inv_holds_obstacle(void)
{
    (void)occ(100u, 0u);
    for (uint32_t t = 66u; t <= 500u; t += 66u) {
        TEST_ASSERT_EQUAL_HEX8(CELL_EVT_NONE, inv(t * MS));
        TEST_ASSERT_EQUAL_INT(CELL_OCCUPIED, g_c.state);
        TEST_ASSERT_EQUAL_UINT16(100u, g_c.dist_cm);
    }
}

/* OCCUPIED + INV resets the CLR counter. */
void test_MAP13_occupied_inv_resets_clr_counter(void)
{
    (void)occ(100u, 0u);
    (void)clr(50u * MS);
    (void)clr(100u * MS);
    (void)inv(150u * MS);
    (void)clr(200u * MS);
    (void)clr(250u * MS);
    TEST_ASSERT_EQUAL_INT(CELL_OCCUPIED, g_c.state);
    (void)clr(300u * MS);
    TEST_ASSERT_EQUAL_INT(CELL_CLEAR_MEASURED, g_c.state);
}

/* OCCUPIED + INV older than HOLD_MAX_MS (500 ms) -> UNKNOWN with a log event. */
void test_MAP13_occupied_inv_after_hold_max_unknown_with_event(void)
{
    (void)occ(100u, 1000u * MS);
    TEST_ASSERT_EQUAL_HEX8(CELL_EVT_NONE, inv(1500u * MS)); /* exactly 500 ms: still held */
    TEST_ASSERT_EQUAL_INT(CELL_OCCUPIED, g_c.state);
    TEST_ASSERT_EQUAL_HEX8(CELL_EVT_HOLD_EXPIRED, inv(1500u * MS + 1u));
    TEST_ASSERT_EQUAL_INT(CELL_UNKNOWN, g_c.state);
}

/* A farther (unconfirmed) OCC is still a valid measurement and restarts the hold timer. */
void test_MAP13_hold_timer_restarts_on_farther_occ(void)
{
    (void)occ(100u, 0u);
    (void)occ(200u, 400u * MS);
    (void)inv(800u * MS);
    TEST_ASSERT_EQUAL_INT(CELL_OCCUPIED, g_c.state);
    TEST_ASSERT_EQUAL_UINT16(100u, g_c.dist_cm);
    (void)inv(901u * MS);
    TEST_ASSERT_EQUAL_INT(CELL_UNKNOWN, g_c.state);
}

/* CLR candidates do not extend the hold: CLR, INV alternation expires at HOLD_MAX_MS. */
void test_MAP13_alternating_clr_inv_expires_hold(void)
{
    (void)occ(100u, 0u);
    uint32_t t = 0u;
    while (g_c.state == CELL_OCCUPIED && t < 2000u) {
        t += 50u;
        (void)((t / 50u) % 2u ? clr(t * MS) : inv(t * MS));
    }
    TEST_ASSERT_EQUAL_INT(CELL_UNKNOWN, g_c.state);
    TEST_ASSERT_TRUE(t > 500u && t <= 600u);
}

void test_MAP13_hold_works_across_time_wraparound(void)
{
    const uint32_t t0 = 0xFFFFFFFFu - 100u * MS;
    (void)occ(100u, t0);
    (void)inv(t0 + 400u * MS); /* wrapped */
    TEST_ASSERT_EQUAL_INT(CELL_OCCUPIED, g_c.state);
    (void)inv(t0 + 501u * MS);
    TEST_ASSERT_EQUAL_INT(CELL_UNKNOWN, g_c.state);
}

/* UNKNOWN / CLEAR_MEASURED + INV -> UNKNOWN. */
void test_MAP13_unknown_inv_stays_unknown(void)
{
    TEST_ASSERT_EQUAL_HEX8(CELL_EVT_NONE, inv(0u));
    TEST_ASSERT_EQUAL_INT(CELL_UNKNOWN, g_c.state);
}

void test_MAP13_clear_inv_becomes_unknown(void)
{
    make_clear(0u);
    TEST_ASSERT_EQUAL_HEX8(CELL_EVT_NONE, inv(10u * MS));
    TEST_ASSERT_EQUAL_INT(CELL_UNKNOWN, g_c.state);
}

/* UNKNOWN + CLR: after CLEAR_CONFIRM_FRAMES in a row -> CLEAR_MEASURED (if enabled). */
void test_MAP13_unknown_clr_3_frames_clear_measured(void)
{
    (void)clr(0u);
    (void)clr(66u * MS);
    TEST_ASSERT_EQUAL_INT(CELL_UNKNOWN, g_c.state);
    TEST_ASSERT_EQUAL_HEX8(CELL_EVT_NONE, clr(133u * MS));
    TEST_ASSERT_EQUAL_INT(CELL_CLEAR_MEASURED, g_c.state);
}

void test_MAP13_unknown_clr_stays_unknown_when_clear_disabled(void)
{
    g_cfg.clear_enabled = false;
    for (unsigned k = 0; k < 300u; k++) { /* also saturates the counter */
        (void)clr(k * MS);
    }
    TEST_ASSERT_EQUAL_INT(CELL_UNKNOWN, g_c.state);
    TEST_ASSERT_EQUAL_UINT8(255u, g_c.clr_count);
}

void test_MAP13_unknown_clr_run_broken_by_inv(void)
{
    (void)clr(0u);
    (void)clr(66u * MS);
    (void)inv(100u * MS);
    (void)clr(133u * MS);
    TEST_ASSERT_EQUAL_INT(CELL_UNKNOWN, g_c.state);
}

void test_MAP13_clear_clr_stays_clear(void)
{
    make_clear(0u);
    (void)clr(100u * MS);
    TEST_ASSERT_EQUAL_INT(CELL_CLEAR_MEASURED, g_c.state);
}

/* After a release a new obstacle at any distance is taken at once. */
void test_MAP13_obstacle_after_release_any_distance(void)
{
    (void)occ(100u, 0u);
    make_clear(10u * MS);
    (void)occ(290u, 100u * MS);
    TEST_ASSERT_EQUAL_UINT16(290u, g_c.dist_cm);
    (void)inv(200u * MS);
    (void)occ(295u, 300u * MS); /* farther again: needs relax */
    TEST_ASSERT_EQUAL_UINT16(290u, g_c.dist_cm);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_MAP13_config_from_params);
    RUN_TEST(test_MAP13_initial_state_unknown);
    RUN_TEST(test_MAP13_unknown_occ_occupied_immediately);
    RUN_TEST(test_MAP13_clear_occ_occupied_immediately);
    RUN_TEST(test_MAP13_occupied_closer_occ_updates_immediately);
    RUN_TEST(test_MAP13_occupied_equal_occ_refreshes_time);
    RUN_TEST(test_MAP13_occupied_farther_occ_relaxes_after_2_frames);
    RUN_TEST(test_MAP13_relax_takes_nearest_confirmation);
    RUN_TEST(test_MAP13_relax_run_broken_by_inv);
    RUN_TEST(test_MAP13_relax_run_broken_by_clr);
    RUN_TEST(test_MAP13_relax_run_broken_by_closer_occ);
    RUN_TEST(test_MAP13_occupied_clr_3_frames_clear_measured);
    RUN_TEST(test_MAP13_occupied_clr_3_frames_unknown_when_clear_disabled);
    RUN_TEST(test_MAP13_occupied_inv_holds_obstacle);
    RUN_TEST(test_MAP13_occupied_inv_resets_clr_counter);
    RUN_TEST(test_MAP13_occupied_inv_after_hold_max_unknown_with_event);
    RUN_TEST(test_MAP13_hold_timer_restarts_on_farther_occ);
    RUN_TEST(test_MAP13_alternating_clr_inv_expires_hold);
    RUN_TEST(test_MAP13_hold_works_across_time_wraparound);
    RUN_TEST(test_MAP13_unknown_inv_stays_unknown);
    RUN_TEST(test_MAP13_clear_inv_becomes_unknown);
    RUN_TEST(test_MAP13_unknown_clr_3_frames_clear_measured);
    RUN_TEST(test_MAP13_unknown_clr_stays_unknown_when_clear_disabled);
    RUN_TEST(test_MAP13_unknown_clr_run_broken_by_inv);
    RUN_TEST(test_MAP13_clear_clr_stays_clear);
    RUN_TEST(test_MAP13_obstacle_after_release_any_distance);
    return UNITY_END();
}
