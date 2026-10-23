#include <string.h>

#include "core/fault_fsm.h"
#include "core/mavlink_tx.h"
#include "unity.h"

#define MS 1000u
#define F_FAILED PARAM_FAULT_BIT_SENSOR_FAILED
#define F_DEGRADED PARAM_FAULT_BIT_SENSOR_DEGRADED

static params_t g_p;
static fault_cfg_t g_cfg;
static fault_fsm_t g_f;
static fault_fc_t g_fc;
static fault_out_t g_out;
static uint32_t g_t;
static uint32_t g_sends;

void setUp(void)
{
    params_set_defaults(&g_p);
    fault_cfg_from_params(&g_cfg, &g_p);
    fault_init(&g_f);
    memset(&g_fc, 0, sizeof g_fc);
    g_fc.mode_known = true;
    g_fc.armed = true;
    g_fc.custom_mode = TOF_COPTER_MODE_AUTO;
    g_fc.hb_count = 10u;
    g_fc.phase = PHASE_FLIGHT;
    g_t = 0u;
    g_sends = 0u;
}

void tearDown(void) {}

static void upd(uint8_t active, uint32_t dt_ms)
{
    g_t += dt_ms * MS;
    fault_update(&g_f, &g_cfg, active, &g_fc, g_t, &g_out);
    g_sends += g_out.send ? 1u : 0u;
}

/* FLT-01, FLT-03: defaults - FAULT_ACTION = BRAKE, trigger mask = FAILED | STALE | LOST. */
void test_FLT03_defaults(void)
{
    TEST_ASSERT_EQUAL_UINT8(PARAM_FAULT_BRAKE, g_cfg.action);
    TEST_ASSERT_EQUAL_UINT8(PARAM_FAULT_BIT_SENSOR_FAILED | PARAM_FAULT_BIT_MAP_STALE | PARAM_FAULT_BIT_MAP_LOST,
                            g_cfg.trigger_mask);
    TEST_ASSERT_TRUE(g_cfg.on_reboot);
}

/* FLT-03: in AUTO / GUIDED / RTL during flight the command goes out on the detecting update
 * (<= 100 ms), BRAKE by default. */
void test_FLT03_command_on_detection(void)
{
    const uint32_t modes[] = {TOF_COPTER_MODE_AUTO, TOF_COPTER_MODE_GUIDED, TOF_COPTER_MODE_RTL};
    const phase_t phases[] = {PHASE_TAKEOFF, PHASE_FLIGHT, PHASE_LANDING, PHASE_UNKNOWN};
    for (unsigned m = 0; m < 3u; m++) {
        for (unsigned ph = 0; ph < 4u; ph++) {
            setUp();
            g_fc.custom_mode = modes[m];
            g_fc.phase = phases[ph];
            upd(0u, 10u);
            TEST_ASSERT_FALSE(g_out.send);
            upd(F_FAILED, 0u);
            TEST_ASSERT_TRUE(g_out.send);
            TEST_ASSERT_EQUAL_UINT32(TOF_COPTER_MODE_BRAKE, g_out.mode);
            TEST_ASSERT_EQUAL_UINT8(0u, g_out.confirmation);
            TEST_ASSERT_TRUE((g_out.events & FAULT_EVT_HOLD_SENT) != 0u);
            TEST_ASSERT_TRUE((g_out.events & FAULT_EVT_EPISODE_START) != 0u);
        }
    }
    setUp();
    g_cfg.action = PARAM_FAULT_LOITER;
    upd(F_FAILED, 0u);
    TEST_ASSERT_EQUAL_UINT32(TOF_COPTER_MODE_LOITER, g_out.mode);
}

/* FLT-05: never in GROUND / ARMED_GROUND / GROUND_RECOVER, never disarmed, never with an unknown mode,
 * never in modes outside AUTO / GUIDED / RTL, never with FAULT_ACTION = NONE. */
void test_FLT05_no_command_on_ground_disarmed_manual(void)
{
    const phase_t ground[] = {PHASE_GROUND, PHASE_ARMED_GROUND, PHASE_GROUND_RECOVER};
    for (unsigned k = 0; k < 3u; k++) {
        setUp();
        g_fc.phase = ground[k];
        for (int i = 0; i < 20; i++) {
            upd(0xFFu, 100u);
        }
        TEST_ASSERT_EQUAL_UINT32(0u, g_sends);
    }
    setUp();
    g_fc.armed = false;
    for (int i = 0; i < 20; i++) {
        upd(0xFFu, 100u);
    }
    TEST_ASSERT_EQUAL_UINT32(0u, g_sends);

    const uint32_t manual[] = {0u /* STABILIZE */, 2u /* ALT_HOLD */, TOF_COPTER_MODE_LOITER, 16u /* POSHOLD */,
                               1u /* ACRO */, TOF_COPTER_MODE_BRAKE};
    for (unsigned k = 0; k < sizeof manual / sizeof manual[0]; k++) {
        setUp();
        g_fc.custom_mode = manual[k];
        for (int i = 0; i < 20; i++) {
            upd(0xFFu, 100u);
        }
        TEST_ASSERT_EQUAL_UINT32(0u, g_sends);
    }
    setUp();
    g_fc.mode_known = false;
    g_fc.phase = PHASE_UNKNOWN;
    for (int i = 0; i < 20; i++) {
        upd(0xFFu, 100u);
    }
    TEST_ASSERT_EQUAL_UINT32(0u, g_sends);
    setUp();
    g_cfg.action = PARAM_FAULT_NONE;
    for (int i = 0; i < 20; i++) {
        upd(0xFFu, 100u);
    }
    TEST_ASSERT_EQUAL_UINT32(0u, g_sends);
}

/* FLT-03: a class outside FAULT_TRIGGER_MASK (DEGRADED by default) does not command. */
void test_FLT03_class_outside_mask(void)
{
    for (int i = 0; i < 20; i++) {
        upd(F_DEGRADED | PARAM_FAULT_BIT_ATT_ALT_LOST | PARAM_FAULT_BIT_INTERNAL, 100u);
    }
    TEST_ASSERT_EQUAL_UINT32(0u, g_sends);
    TEST_ASSERT_FALSE(g_f.episode);
}

/* FLT-05: the command starts as soon as the conditions hold (here: takeoff after the fault). */
void test_FLT05_command_when_phase_allows(void)
{
    g_fc.phase = PHASE_ARMED_GROUND;
    upd(F_FAILED, 10u);
    upd(F_FAILED, 1000u);
    TEST_ASSERT_EQUAL_UINT32(0u, g_sends);
    g_fc.phase = PHASE_TAKEOFF;
    upd(F_FAILED, 10u);
    TEST_ASSERT_TRUE(g_out.send);
}

/* FLT-03, FLT-03a: first transmission and up to 5 repeats every 500 ms (6 in total); FLT-06: HOLD
 * REJECTED 500 ms after the last one without a mode change; no commands afterwards. */
void test_FLT03a_retry_500ms_x5_then_rejected(void)
{
    upd(F_FAILED, 0u);
    TEST_ASSERT_EQUAL_UINT32(1u, g_sends);
    for (uint8_t n = 1u; n < FAULT_MAX_SENDS; n++) {
        upd(F_FAILED, 499u);
        TEST_ASSERT_FALSE(g_out.send);
        upd(F_FAILED, 1u);
        TEST_ASSERT_TRUE(g_out.send);
        TEST_ASSERT_EQUAL_UINT8(n, g_out.confirmation);
        g_fc.hb_count++; /* heartbeats keep coming, mode stays AUTO */
    }
    TEST_ASSERT_EQUAL_UINT32(6u, g_sends);
    upd(F_FAILED, 499u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_out.events & FAULT_EVT_HOLD_REJECTED);
    upd(F_FAILED, 1u);
    TEST_ASSERT_TRUE((g_out.events & FAULT_EVT_HOLD_REJECTED) != 0u);
    TEST_ASSERT_EQUAL_INT(FAULT_SERIES_REJECTED, g_f.series);
    for (int i = 0; i < 50; i++) {
        upd(F_FAILED, 100u);
    }
    TEST_ASSERT_EQUAL_UINT32(6u, g_sends);
}

/* FLT-03: success only by the mode in a HEARTBEAT received after the command. A mode field that
 * did not come with a new HEARTBEAT (or a COMMAND_ACK, which the automaton does not even see) is not
 * a success. */
void test_FLT03_success_only_by_heartbeat(void)
{
    upd(F_FAILED, 0u);
    g_fc.custom_mode = TOF_COPTER_MODE_BRAKE; /* no new HEARTBEAT yet */
    upd(F_FAILED, 100u);
    TEST_ASSERT_EQUAL_INT(FAULT_SERIES_SENDING, g_f.series);
    g_fc.custom_mode = TOF_COPTER_MODE_AUTO;
    g_fc.hb_count++; /* HEARTBEAT still in AUTO */
    upd(F_FAILED, 100u);
    TEST_ASSERT_EQUAL_INT(FAULT_SERIES_SENDING, g_f.series);
    g_fc.custom_mode = TOF_COPTER_MODE_BRAKE;
    g_fc.hb_count++;
    upd(F_FAILED, 100u);
    TEST_ASSERT_TRUE((g_out.events & FAULT_EVT_HOLD_OK) != 0u);
    TEST_ASSERT_EQUAL_INT(FAULT_SERIES_SUCCEEDED, g_f.series);
    for (int i = 0; i < 50; i++) {
        upd(F_FAILED, 100u);
    }
    TEST_ASSERT_EQUAL_UINT32(1u, g_sends);
}

/* FLT-04: one successful transfer per episode; the pilot returning AUTO under the same fault gets
 * only a warning. A new episode commands again. */
void test_FLT04_one_transfer_per_episode_pilot_resumes(void)
{
    upd(F_FAILED, 0u);
    g_fc.custom_mode = TOF_COPTER_MODE_BRAKE;
    g_fc.hb_count++;
    upd(F_FAILED, 100u);
    TEST_ASSERT_EQUAL_INT(FAULT_SERIES_SUCCEEDED, g_f.series);
    g_fc.custom_mode = TOF_COPTER_MODE_AUTO; /* pilot */
    g_fc.hb_count++;
    upd(F_FAILED, 100u);
    TEST_ASSERT_FALSE(g_out.send);
    TEST_ASSERT_TRUE((g_out.events & FAULT_EVT_PILOT_RESUMED) != 0u);
    for (int i = 0; i < 30; i++) {
        upd(F_FAILED, 100u);
        TEST_ASSERT_EQUAL_UINT8(0u, g_out.events & FAULT_EVT_PILOT_RESUMED); /* once per resumption */
    }
    TEST_ASSERT_EQUAL_UINT32(1u, g_sends);
    /* The fault clears: episode ends. A new fault is a new episode. */
    upd(0u, 100u);
    TEST_ASSERT_TRUE((g_out.events & FAULT_EVT_EPISODE_END) != 0u);
    TEST_ASSERT_FALSE(g_f.episode);
    upd(PARAM_FAULT_BIT_MAP_LOST, 100u);
    TEST_ASSERT_TRUE(g_out.send);
    TEST_ASSERT_EQUAL_UINT32(2u, g_sends);
    TEST_ASSERT_EQUAL_UINT32(2u, g_f.episodes_total);
}

/* FLT-04: a change of fault class inside the episode is the same episode (no new series). */
void test_FLT04_episode_spans_class_changes(void)
{
    upd(F_FAILED, 0u);
    g_fc.custom_mode = TOF_COPTER_MODE_BRAKE;
    g_fc.hb_count++;
    upd(F_FAILED, 100u);
    g_fc.custom_mode = TOF_COPTER_MODE_AUTO;
    g_fc.hb_count++;
    upd(F_FAILED | PARAM_FAULT_BIT_MAP_LOST, 100u);
    upd(PARAM_FAULT_BIT_MAP_LOST, 100u);
    TEST_ASSERT_EQUAL_UINT32(1u, g_sends);
}

/* FLT-04: the pilot taking another mode during the retries stops the series; AUTO again is only
 * warned about. */
void test_FLT04_pilot_takes_over_during_retries(void)
{
    upd(F_FAILED, 0u);
    g_fc.custom_mode = 2u; /* ALT_HOLD by the pilot */
    g_fc.hb_count++;
    upd(F_FAILED, 100u);
    TEST_ASSERT_EQUAL_INT(FAULT_SERIES_ABORTED, g_f.series);
    g_fc.custom_mode = TOF_COPTER_MODE_AUTO;
    g_fc.hb_count++;
    upd(F_FAILED, 600u);
    TEST_ASSERT_FALSE(g_out.send);
    TEST_ASSERT_TRUE((g_out.events & FAULT_EVT_PILOT_RESUMED) != 0u);
    TEST_ASSERT_EQUAL_UINT32(1u, g_sends);
}

/* FLT-03: lost FC link pauses the retries; they continue when it is back. */
void test_FLT03_link_loss_pauses(void)
{
    upd(F_FAILED, 0u);
    g_fc.mode_known = false;
    g_fc.phase = PHASE_UNKNOWN;
    upd(F_FAILED, 600u);
    upd(F_FAILED, 600u);
    TEST_ASSERT_EQUAL_UINT32(1u, g_sends);
    TEST_ASSERT_EQUAL_INT(FAULT_SERIES_SENDING, g_f.series);
    g_fc.mode_known = true;
    upd(F_FAILED, 10u);
    TEST_ASSERT_TRUE(g_out.send);
    TEST_ASSERT_EQUAL_UINT8(1u, g_out.confirmation);
}

/* FLT-04: the fault vanishing during the retries ends the episode and the retries. */
void test_FLT04_fault_gone_stops_retries(void)
{
    upd(F_FAILED, 0u);
    upd(0u, 600u);
    TEST_ASSERT_FALSE(g_out.send);
    TEST_ASSERT_FALSE(g_f.episode);
    upd(0u, 600u);
    TEST_ASSERT_EQUAL_UINT32(1u, g_sends);
}

/* FLT-08: first FC HEARTBEAT armed in AUTO after a restart - hold command and event; once. */
void test_FLT08_rebooted_in_flight(void)
{
    g_fc.first_hb = true;
    upd(0u, 0u);
    TEST_ASSERT_TRUE((g_out.events & FAULT_EVT_REBOOTED_IN_FLIGHT) != 0u);
    TEST_ASSERT_TRUE(g_out.send);
    TEST_ASSERT_EQUAL_UINT32(TOF_COPTER_MODE_BRAKE, g_out.mode);
    g_fc.first_hb = false;
    g_fc.custom_mode = TOF_COPTER_MODE_BRAKE;
    g_fc.hb_count++;
    upd(0u, 100u);
    TEST_ASSERT_TRUE((g_out.events & FAULT_EVT_HOLD_OK) != 0u);
    TEST_ASSERT_TRUE((g_out.events & FAULT_EVT_EPISODE_END) != 0u); /* no other fault */
    g_fc.custom_mode = TOF_COPTER_MODE_AUTO;
    g_fc.hb_count++;
    for (int i = 0; i < 20; i++) {
        upd(0u, 100u);
    }
    TEST_ASSERT_EQUAL_UINT32(1u, g_sends);
}

/* FLT-08: not armed or not in AUTO/GUIDED/RTL at the first HEARTBEAT - nothing; FAULT_ON_REBOOT = 0 -
 * event only. */
void test_FLT08_conditions(void)
{
    g_fc.first_hb = true;
    g_fc.armed = false;
    upd(0u, 0u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_out.events);
    setUp();
    g_fc.first_hb = true;
    g_fc.custom_mode = TOF_COPTER_MODE_LOITER;
    upd(0u, 0u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_out.events);
    setUp();
    g_cfg.on_reboot = false;
    g_fc.first_hb = true;
    upd(0u, 0u);
    TEST_ASSERT_TRUE((g_out.events & FAULT_EVT_REBOOTED_IN_FLIGHT) != 0u);
    TEST_ASSERT_FALSE(g_out.send);
    TEST_ASSERT_FALSE(g_f.episode);
    /* FAULT_ACTION = NONE: the latch is dropped, no command. */
    setUp();
    g_cfg.action = PARAM_FAULT_NONE;
    g_fc.first_hb = true;
    upd(0u, 0u);
    TEST_ASSERT_FALSE(g_out.send);
    upd(0u, 100u);
    TEST_ASSERT_FALSE(g_f.episode);
}

/* MAP-16: GROUND_RECOVER resets the episode; a persisting fault starts a new one that commands
 * again after the next takeoff. */
void test_FLT04_ground_recover_resets_episode(void)
{
    upd(F_FAILED, 0u);
    g_fc.custom_mode = TOF_COPTER_MODE_BRAKE;
    g_fc.hb_count++;
    upd(F_FAILED, 100u);
    g_fc.phase = PHASE_GROUND_RECOVER;
    fault_reset_episode(&g_f);
    upd(F_FAILED, 10u);
    TEST_ASSERT_TRUE((g_out.events & FAULT_EVT_EPISODE_START) != 0u);
    TEST_ASSERT_FALSE(g_out.send);
    g_fc.phase = PHASE_TAKEOFF;
    g_fc.custom_mode = TOF_COPTER_MODE_AUTO;
    upd(F_FAILED, 10u);
    TEST_ASSERT_TRUE(g_out.send);
}

/* FLT-10: FAULT_WALL is 0 by default; when applied, sectors of a FAILED sensor get min_distance. */
void test_FLT10_virtual_wall(void)
{
    TEST_ASSERT_EQUAL_UINT32(0u, params_get_u32(&g_p, PARAM_FAULT_WALL));
    static fusion_t fu;
    fusion_init(&fu, &g_p);
    uint16_t d[TOF_NUM_SECTORS];
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        d[k] = 65535u;
    }
    const sensor_health_t ok[TOF_NUM_SENSORS] = {SENSOR_OK, SENSOR_DEGRADED, SENSOR_OK};
    fault_wall_apply(&fu, ok, 4u, d);
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        TEST_ASSERT_EQUAL_UINT16(65535u, d[k]);
    }
    const sensor_health_t r_failed[TOF_NUM_SENSORS] = {SENSOR_OK, SENSOR_OK, SENSOR_FAILED};
    fault_wall_apply(&fu, r_failed, 4u, d);
    uint32_t walls = 0u;
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        const bool r_sector = fu.sector_zones[k][TOF_SENSOR_R] != 0u;
        TEST_ASSERT_EQUAL_UINT16(r_sector ? 4u : 65535u, d[k]);
        walls += r_sector ? 1u : 0u;
    }
    TEST_ASSERT_TRUE(walls >= 9u); /* 45 deg + tolerance */
    TEST_ASSERT_EQUAL_UINT16(4u, d[6]); /* +30 deg */
}

/* FLT-03: a command that could not be queued is not counted and goes out on the next pass. */
void test_FLT03_unqueued_command_retried(void)
{
    upd(F_FAILED, 0u);
    TEST_ASSERT_TRUE(g_out.send);
    fault_send_failed(&g_f);
    upd(F_FAILED, 5u);
    TEST_ASSERT_TRUE(g_out.send);
    TEST_ASSERT_EQUAL_UINT8(0u, g_out.confirmation);
    TEST_ASSERT_TRUE((g_out.events & FAULT_EVT_HOLD_SENT) != 0u);
    TEST_ASSERT_EQUAL_UINT8(1u, g_f.sends);
    fault_init(&g_f);
    fault_send_failed(&g_f); /* nothing sent: no effect */
    TEST_ASSERT_EQUAL_UINT8(0u, g_f.sends);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_FLT03_defaults);
    RUN_TEST(test_FLT03_command_on_detection);
    RUN_TEST(test_FLT05_no_command_on_ground_disarmed_manual);
    RUN_TEST(test_FLT03_class_outside_mask);
    RUN_TEST(test_FLT05_command_when_phase_allows);
    RUN_TEST(test_FLT03a_retry_500ms_x5_then_rejected);
    RUN_TEST(test_FLT03_success_only_by_heartbeat);
    RUN_TEST(test_FLT04_one_transfer_per_episode_pilot_resumes);
    RUN_TEST(test_FLT04_episode_spans_class_changes);
    RUN_TEST(test_FLT04_pilot_takes_over_during_retries);
    RUN_TEST(test_FLT03_link_loss_pauses);
    RUN_TEST(test_FLT04_fault_gone_stops_retries);
    RUN_TEST(test_FLT08_rebooted_in_flight);
    RUN_TEST(test_FLT08_conditions);
    RUN_TEST(test_FLT04_ground_recover_resets_episode);
    RUN_TEST(test_FLT10_virtual_wall);
    RUN_TEST(test_FLT03_unqueued_command_retried);
    return UNITY_END();
}
