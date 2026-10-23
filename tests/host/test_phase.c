#include <string.h>

#include "core/phase.h"
#include "unity.h"

#define MS 1000u

static params_t g_p;
static phase_cfg_t g_cfg;
static phase_fsm_t g_f;
static phase_input_t g_in;
static uint32_t g_t;

void setUp(void)
{
    params_set_defaults(&g_p);
    phase_cfg_from_params(&g_cfg, &g_p);
    phase_init(&g_f, 0u);
    memset(&g_in, 0, sizeof g_in);
    g_in.hb_valid = true;
    g_in.ess_valid = true;
    g_in.landed_state = PHASE_LS_ON_GROUND;
    g_in.h_valid = true;
    g_in.h_m = 0.2f;
    g_t = 0u;
}

void tearDown(void) {}

static uint8_t step(uint32_t dt_ms)
{
    g_t += dt_ms * MS;
    return phase_update(&g_f, &g_cfg, &g_in, g_t);
}

static void fly(float h)
{
    g_in.armed = true;
    g_in.landed_state = PHASE_LS_IN_AIR;
    g_in.h_m = h;
}

/* Reaches FLIGHT through ARMED_GROUND and TAKEOFF. */
static void to_flight(void)
{
    g_in.armed = true;
    (void)step(10u);
    fly(2.0f);
    (void)step(10u);
    (void)step(1000u);
    TEST_ASSERT_EQUAL_INT(PHASE_FLIGHT, g_f.phase);
}

/* MAP-16: PHASE_UNKNOWN at start, until FC data arrive. */
void test_MAP16_unknown_at_start(void)
{
    TEST_ASSERT_EQUAL_INT(PHASE_UNKNOWN, g_f.phase);
    g_in.hb_valid = false;
    TEST_ASSERT_EQUAL_UINT8(PHASE_EVT_NONE, step(10u));
    TEST_ASSERT_EQUAL_INT(PHASE_UNKNOWN, g_f.phase);
}

/* MAP-16: disarmed -> GROUND (also with LS other than ON_GROUND). */
void test_MAP16_disarmed_is_ground(void)
{
    TEST_ASSERT_EQUAL_UINT8(PHASE_EVT_CHANGED, step(10u));
    TEST_ASSERT_EQUAL_INT(PHASE_GROUND, g_f.phase);
    g_in.landed_state = PHASE_LS_IN_AIR;
    (void)step(10u);
    TEST_ASSERT_EQUAL_INT(PHASE_GROUND, g_f.phase);
}

/* MAP-16: armed + ON_GROUND -> ARMED_GROUND; disarm -> GROUND without reset (no flight). */
void test_MAP16_ground_to_armed_ground_and_back(void)
{
    (void)step(10u);
    g_in.armed = true;
    TEST_ASSERT_EQUAL_UINT8(PHASE_EVT_CHANGED, step(10u));
    TEST_ASSERT_EQUAL_INT(PHASE_ARMED_GROUND, g_f.phase);
    g_in.armed = false;
    TEST_ASSERT_EQUAL_UINT8(PHASE_EVT_CHANGED, step(10u));
    TEST_ASSERT_EQUAL_INT(PHASE_GROUND, g_f.phase);
}

/* MAP-16: ARMED_GROUND -> TAKEOFF on LS = TAKEOFF. */
void test_MAP16_takeoff_on_ls_takeoff(void)
{
    g_in.armed = true;
    (void)step(10u);
    g_in.landed_state = PHASE_LS_TAKEOFF;
    g_in.h_m = 3.0f; /* height does not matter for LS = TAKEOFF */
    (void)step(2000u);
    TEST_ASSERT_EQUAL_INT(PHASE_TAKEOFF, g_f.phase);
}

/* MAP-16: ARMED_GROUND -> TAKEOFF on LS = IN_AIR below PHASE_H_FLIGHT_M. */
void test_MAP16_takeoff_on_in_air_low(void)
{
    g_in.armed = true;
    (void)step(10u);
    fly(1.0f);
    (void)step(10u);
    TEST_ASSERT_EQUAL_INT(PHASE_TAKEOFF, g_f.phase);
    (void)step(5000u);
    TEST_ASSERT_EQUAL_INT(PHASE_TAKEOFF, g_f.phase);
}

/* MAP-16: TAKEOFF -> FLIGHT after h_tof >= PHASE_H_FLIGHT_M for 1 s, not earlier. */
void test_MAP16_takeoff_to_flight_after_1s(void)
{
    g_in.armed = true;
    (void)step(10u);
    fly(1.5f);
    (void)step(10u); /* timer starts */
    TEST_ASSERT_EQUAL_INT(PHASE_TAKEOFF, g_f.phase);
    (void)step(999u);
    TEST_ASSERT_EQUAL_INT(PHASE_TAKEOFF, g_f.phase);
    TEST_ASSERT_EQUAL_UINT8(PHASE_EVT_CHANGED, step(1u));
    TEST_ASSERT_EQUAL_INT(PHASE_FLIGHT, g_f.phase);
}

/* MAP-16a: "h_tof >= PHASE_H_FLIGHT_M during 1 s" - any dip below the threshold restarts the
 * confirmation (no height hysteresis). */
void test_MAP16_flight_confirmation_restarts_on_dip(void)
{
    g_in.armed = true;
    (void)step(10u);
    fly(1.6f);
    (void)step(10u);
    g_in.h_m = 1.49f; /* just below: restart */
    (void)step(500u);
    g_in.h_m = 1.6f;
    (void)step(500u);
    TEST_ASSERT_EQUAL_INT(PHASE_TAKEOFF, g_f.phase);
    (void)step(999u);
    TEST_ASSERT_EQUAL_INT(PHASE_TAKEOFF, g_f.phase);
    (void)step(1u);
    TEST_ASSERT_EQUAL_INT(PHASE_FLIGHT, g_f.phase);
}

/* MAP-16: FLIGHT stays while LS = IN_AIR, whatever the height. */
void test_MAP16_flight_sticky_while_in_air(void)
{
    to_flight();
    g_in.h_m = 0.3f;
    TEST_ASSERT_EQUAL_UINT8(PHASE_EVT_NONE, step(5000u));
    TEST_ASSERT_EQUAL_INT(PHASE_FLIGHT, g_f.phase);
}

/* MAP-16: FLIGHT -> LANDING on LS = LANDING; LANDING -> GROUND_RECOVER (reset) -> ARMED_GROUND ->
 * GROUND. */
void test_MAP16_landing_and_ground_recover(void)
{
    to_flight();
    g_in.landed_state = PHASE_LS_LANDING;
    TEST_ASSERT_EQUAL_UINT8(PHASE_EVT_CHANGED, step(10u));
    TEST_ASSERT_EQUAL_INT(PHASE_LANDING, g_f.phase);
    g_in.landed_state = PHASE_LS_ON_GROUND;
    TEST_ASSERT_EQUAL_UINT8(PHASE_EVT_CHANGED | PHASE_EVT_RESET, step(10u));
    TEST_ASSERT_EQUAL_INT(PHASE_GROUND_RECOVER, g_f.phase);
    TEST_ASSERT_EQUAL_UINT8(PHASE_EVT_CHANGED, step(10u));
    TEST_ASSERT_EQUAL_INT(PHASE_ARMED_GROUND, g_f.phase);
    g_in.armed = false;
    TEST_ASSERT_EQUAL_UINT8(PHASE_EVT_CHANGED, step(10u)); /* no second reset */
    TEST_ASSERT_EQUAL_INT(PHASE_GROUND, g_f.phase);
}

/* MAP-16a: FLIGHT is not left on disarm while LS = IN_AIR; LS = ON_GROUND then recovers to GROUND. */
void test_MAP16a_disarm_in_flight_stays_until_on_ground(void)
{
    to_flight();
    g_in.armed = false;
    TEST_ASSERT_EQUAL_UINT8(PHASE_EVT_NONE, step(10u));
    TEST_ASSERT_EQUAL_INT(PHASE_FLIGHT, g_f.phase);
    g_in.landed_state = PHASE_LS_ON_GROUND;
    TEST_ASSERT_EQUAL_UINT8(PHASE_EVT_CHANGED | PHASE_EVT_RESET, step(10u));
    TEST_ASSERT_EQUAL_INT(PHASE_GROUND_RECOVER, g_f.phase);
    (void)step(10u);
    TEST_ASSERT_EQUAL_INT(PHASE_GROUND, g_f.phase);
}

/* MAP-15b: IN_AIR without a valid height - FLIGHT at once; LS = TAKEOFF stays TAKEOFF. */
void test_MAP15b_in_air_without_height_is_flight(void)
{
    g_in.armed = true;
    (void)step(10u);
    g_in.landed_state = PHASE_LS_TAKEOFF;
    g_in.h_valid = false;
    (void)step(10u);
    TEST_ASSERT_EQUAL_INT(PHASE_TAKEOFF, g_f.phase);
    g_in.landed_state = PHASE_LS_IN_AIR;
    TEST_ASSERT_EQUAL_UINT8(PHASE_EVT_CHANGED, step(10u));
    TEST_ASSERT_EQUAL_INT(PHASE_FLIGHT, g_f.phase);
    /* The height coming back does not leave FLIGHT. */
    g_in.h_valid = true;
    g_in.h_m = 0.5f;
    (void)step(10u);
    TEST_ASSERT_EQUAL_INT(PHASE_FLIGHT, g_f.phase);
}

/* MAP-15b: the height lost during the TAKEOFF confirmation - FLIGHT. */
void test_MAP15b_height_lost_during_takeoff(void)
{
    g_in.armed = true;
    (void)step(10u);
    fly(1.0f);
    (void)step(10u);
    TEST_ASSERT_EQUAL_INT(PHASE_TAKEOFF, g_f.phase);
    g_in.h_valid = false;
    (void)step(10u);
    TEST_ASSERT_EQUAL_INT(PHASE_FLIGHT, g_f.phase);
}

/* MAP-16: TAKEOFF aborted (LS back to ON_GROUND) also recovers. */
void test_MAP16_takeoff_aborted_recovers(void)
{
    g_in.armed = true;
    (void)step(10u);
    g_in.landed_state = PHASE_LS_TAKEOFF;
    (void)step(10u);
    g_in.landed_state = PHASE_LS_ON_GROUND;
    TEST_ASSERT_EQUAL_UINT8(PHASE_EVT_CHANGED | PHASE_EVT_RESET, step(10u));
    TEST_ASSERT_EQUAL_INT(PHASE_GROUND_RECOVER, g_f.phase);
}

/* MAP-16: no HEARTBEAT or EXTENDED_SYS_STATE (> 3 s, decided by mavlink_rx) -> PHASE_UNKNOWN from any
 * phase; LS undefined too. */
void test_MAP16_unknown_on_timeouts(void)
{
    to_flight();
    g_in.hb_valid = false;
    TEST_ASSERT_EQUAL_UINT8(PHASE_EVT_CHANGED, step(10u));
    TEST_ASSERT_EQUAL_INT(PHASE_UNKNOWN, g_f.phase);

    setUp();
    (void)step(10u);
    g_in.ess_valid = false;
    (void)step(10u);
    TEST_ASSERT_EQUAL_INT(PHASE_UNKNOWN, g_f.phase);

    setUp();
    g_in.armed = true;
    g_in.landed_state = PHASE_LS_UNDEFINED;
    (void)step(10u);
    TEST_ASSERT_EQUAL_INT(PHASE_UNKNOWN, g_f.phase);
}

/* MAP-16a: back from PHASE_UNKNOWN in the air after FLIGHT - FLIGHT only by the entry rule: 1 s at
 * PHASE_H_FLIGHT_M, or at once without a valid height (MAP-15b). */
void test_MAP16a_unknown_back_after_flight(void)
{
    to_flight();
    g_in.ess_valid = false;
    (void)step(10u);
    g_in.ess_valid = true;
    g_in.h_m = 2.0f;
    (void)step(10u);
    TEST_ASSERT_EQUAL_INT(PHASE_TAKEOFF, g_f.phase);
    (void)step(1000u);
    TEST_ASSERT_EQUAL_INT(PHASE_FLIGHT, g_f.phase);
    g_in.hb_valid = false;
    (void)step(10u);
    g_in.hb_valid = true;
    g_in.h_valid = false;
    (void)step(10u);
    TEST_ASSERT_EQUAL_INT(PHASE_FLIGHT, g_f.phase);
}

/* MAP-16: back from PHASE_UNKNOWN in the air after TAKEOFF -> TAKEOFF with a new confirmation. */
void test_MAP16_unknown_back_to_takeoff(void)
{
    g_in.armed = true;
    (void)step(10u);
    fly(1.0f);
    (void)step(10u);
    g_in.hb_valid = false;
    (void)step(10u);
    g_in.hb_valid = true;
    g_in.h_m = 2.0f;
    (void)step(10u);
    TEST_ASSERT_EQUAL_INT(PHASE_TAKEOFF, g_f.phase);
    (void)step(1000u);
    TEST_ASSERT_EQUAL_INT(PHASE_FLIGHT, g_f.phase);
}

/* MAP-16: a gap while airborne that ends on the ground still resets (GROUND_RECOVER). */
void test_MAP16_unknown_after_flight_ends_on_ground(void)
{
    to_flight();
    g_in.hb_valid = false;
    (void)step(10u);
    g_in.hb_valid = true;
    g_in.landed_state = PHASE_LS_ON_GROUND;
    TEST_ASSERT_EQUAL_UINT8(PHASE_EVT_CHANGED | PHASE_EVT_RESET, step(10u));
    TEST_ASSERT_EQUAL_INT(PHASE_GROUND_RECOVER, g_f.phase);
}

/* MAP-16: go-around - LANDING back to IN_AIR goes through TAKEOFF and its 1 s confirmation. */
void test_MAP16_landing_go_around(void)
{
    to_flight();
    g_in.landed_state = PHASE_LS_LANDING;
    (void)step(10u);
    g_in.landed_state = PHASE_LS_IN_AIR;
    g_in.h_m = 2.0f;
    (void)step(10u);
    TEST_ASSERT_EQUAL_INT(PHASE_TAKEOFF, g_f.phase);
    (void)step(1000u);
    TEST_ASSERT_EQUAL_INT(PHASE_FLIGHT, g_f.phase);
}

/* MAP-16a: FLIGHT is left only on LS = LANDING / ON_GROUND or PHASE_UNKNOWN - LS = TAKEOFF keeps it. */
void test_MAP16a_flight_kept_on_ls_takeoff(void)
{
    to_flight();
    g_in.landed_state = PHASE_LS_TAKEOFF;
    (void)step(10u);
    TEST_ASSERT_EQUAL_INT(PHASE_FLIGHT, g_f.phase);
}

/* MAP-17 table 5.4, column "GROUND_SUSPECT points": 65535 (false) or as an obstacle (true). */
void test_MAP17_export_policy_table(void)
{
    TEST_ASSERT_FALSE(phase_export_ground(PHASE_GROUND, PARAM_GROUND_KEEP, true));
    TEST_ASSERT_FALSE(phase_export_ground(PHASE_ARMED_GROUND, PARAM_GROUND_KEEP, true));
    TEST_ASSERT_FALSE(phase_export_ground(PHASE_TAKEOFF, PARAM_GROUND_KEEP, true));
    TEST_ASSERT_TRUE(phase_export_ground(PHASE_FLIGHT, PARAM_GROUND_KEEP, true));
    TEST_ASSERT_TRUE(phase_export_ground(PHASE_FLIGHT, PARAM_GROUND_KEEP, false));
    TEST_ASSERT_FALSE(phase_export_ground(PHASE_LANDING, PARAM_GROUND_KEEP, true));
    TEST_ASSERT_FALSE(phase_export_ground(PHASE_GROUND_RECOVER, PARAM_GROUND_KEEP, true));
    TEST_ASSERT_TRUE(phase_export_ground(PHASE_UNKNOWN, PARAM_GROUND_KEEP, false));
}

/* MAP-07a: without TOF_ALLOW_GROUND_DROP the value DROP never drops in FLIGHT (and the parameter
 * itself is rejected at load, test_params). */
void test_MAP07a_drop_inactive_without_build_flag(void)
{
    TEST_ASSERT_TRUE(phase_export_ground(PHASE_FLIGHT, PARAM_GROUND_DROP, true));
    TEST_ASSERT_TRUE(phase_export_ground(PHASE_UNKNOWN, PARAM_GROUND_DROP, true));
}

/* MAP-17 table 5.4, column "hold command" (FLT-03, FLT-05). */
void test_MAP17_hold_allowed_table(void)
{
    TEST_ASSERT_FALSE(phase_hold_allowed(PHASE_GROUND, true));
    TEST_ASSERT_FALSE(phase_hold_allowed(PHASE_ARMED_GROUND, true));
    TEST_ASSERT_TRUE(phase_hold_allowed(PHASE_TAKEOFF, true));
    TEST_ASSERT_TRUE(phase_hold_allowed(PHASE_FLIGHT, true));
    TEST_ASSERT_TRUE(phase_hold_allowed(PHASE_LANDING, true));
    TEST_ASSERT_FALSE(phase_hold_allowed(PHASE_GROUND_RECOVER, true));
    TEST_ASSERT_TRUE(phase_hold_allowed(PHASE_UNKNOWN, true));
    TEST_ASSERT_FALSE(phase_hold_allowed(PHASE_UNKNOWN, false));
}

/* DIA-01, DIA-04: names and ground phases. */
void test_DIA04_ground_phases_and_names(void)
{
    TEST_ASSERT_TRUE(phase_is_ground(PHASE_GROUND));
    TEST_ASSERT_TRUE(phase_is_ground(PHASE_ARMED_GROUND));
    TEST_ASSERT_TRUE(phase_is_ground(PHASE_GROUND_RECOVER));
    TEST_ASSERT_FALSE(phase_is_ground(PHASE_TAKEOFF));
    TEST_ASSERT_FALSE(phase_is_ground(PHASE_UNKNOWN));
    TEST_ASSERT_EQUAL_STRING("FLT", phase_name(PHASE_FLIGHT));
    TEST_ASSERT_EQUAL_STRING("UNK", phase_name(PHASE_COUNT));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_MAP16_unknown_at_start);
    RUN_TEST(test_MAP16_disarmed_is_ground);
    RUN_TEST(test_MAP16_ground_to_armed_ground_and_back);
    RUN_TEST(test_MAP16_takeoff_on_ls_takeoff);
    RUN_TEST(test_MAP16_takeoff_on_in_air_low);
    RUN_TEST(test_MAP16_takeoff_to_flight_after_1s);
    RUN_TEST(test_MAP16_flight_confirmation_restarts_on_dip);
    RUN_TEST(test_MAP16_flight_sticky_while_in_air);
    RUN_TEST(test_MAP16_landing_and_ground_recover);
    RUN_TEST(test_MAP16a_disarm_in_flight_stays_until_on_ground);
    RUN_TEST(test_MAP15b_in_air_without_height_is_flight);
    RUN_TEST(test_MAP15b_height_lost_during_takeoff);
    RUN_TEST(test_MAP16_takeoff_aborted_recovers);
    RUN_TEST(test_MAP16_unknown_on_timeouts);
    RUN_TEST(test_MAP16a_unknown_back_after_flight);
    RUN_TEST(test_MAP16_unknown_back_to_takeoff);
    RUN_TEST(test_MAP16_unknown_after_flight_ends_on_ground);
    RUN_TEST(test_MAP16_landing_go_around);
    RUN_TEST(test_MAP16a_flight_kept_on_ls_takeoff);
    RUN_TEST(test_MAP17_export_policy_table);
    RUN_TEST(test_MAP07a_drop_inactive_without_build_flag);
    RUN_TEST(test_MAP17_hold_allowed_table);
    RUN_TEST(test_DIA04_ground_phases_and_names);
    return UNITY_END();
}
