#include <string.h>

#include "ardupilotmega/mavlink.h"
#include "core/events.h"
#include "core/version.h"
#include "unity.h"

#define MS 1000u
#define NONE EV_DIST_NONE

static params_t g_p;
static ev_cfg_t g_cfg;
static ev_t g_e;
static uint8_t g_sev;
static char g_text[EV_TEXT_LEN + 1u];

void setUp(void)
{
    params_set_defaults(&g_p);
    ev_cfg_from_params(&g_cfg, &g_p);
    ev_init(&g_e);
}

void tearDown(void) {}

static bool next(uint32_t t_ms)
{
    memset(g_text, 0, sizeof g_text);
    return ev_next_statustext(&g_e, t_ms * MS, &g_sev, g_text);
}

static void obst(phase_t ph, uint16_t exported, uint16_t real)
{
    ev_obstacle(&g_e, &g_cfg, ph, exported, 15, real, -5);
}

/* DIA-05: start thresholds 250 / 120 cm, hysteresis 30 cm. */
void test_DIA05_thresholds(void)
{
    TEST_ASSERT_EQUAL_UINT16(250u, g_cfg.warn_cm);
    TEST_ASSERT_EQUAL_UINT16(120u, g_cfg.crit_cm);
    TEST_ASSERT_EQUAL_UINT16(30u, g_cfg.hyst_cm);
}

/* DIA-03: severities are MAV_SEVERITY values. */
void test_DIA03_severity_values(void)
{
    TEST_ASSERT_EQUAL_UINT8(MAV_SEVERITY_CRITICAL, EV_SEV_CRITICAL);
    TEST_ASSERT_EQUAL_UINT8(MAV_SEVERITY_WARNING, EV_SEV_WARNING);
    TEST_ASSERT_EQUAL_UINT8(MAV_SEVERITY_INFO, EV_SEV_INFO);
}

/* DIA-03: every event has a "TOF" text of <= 50 characters and its severity. */
void test_DIA03_all_texts(void)
{
    static const struct {
        int ch;
        uint8_t st;
        const char *text;
        uint8_t sev;
    } chans[] = {
        {EV_CH_SENSOR_L, 1u, "TOF SENSOR L LOST", EV_SEV_CRITICAL},
        {EV_CH_SENSOR_C, 1u, "TOF SENSOR C LOST", EV_SEV_CRITICAL},
        {EV_CH_SENSOR_R, 1u, "TOF SENSOR R LOST", EV_SEV_CRITICAL},
        {EV_CH_STALE, 1u, "TOF DATA STALE", EV_SEV_WARNING},
        {EV_CH_MAP_LOST, 1u, "TOF MAP LOST", EV_SEV_CRITICAL},
        {EV_CH_FC_LINK, 1u, "TOF FC LINK LOST", EV_SEV_WARNING},
        {EV_CH_PHASE, 1u, "TOF PHASE UNKNOWN", EV_SEV_WARNING},
    };
    uint32_t t = 0u;
    for (unsigned k = 0; k < sizeof chans / sizeof chans[0]; k++) {
        ev_set_state(&g_e, (ev_channel_t)chans[k].ch, chans[k].st);
        t += 2000u;
        TEST_ASSERT_TRUE(next(t));
        TEST_ASSERT_EQUAL_STRING(chans[k].text, g_text);
        TEST_ASSERT_EQUAL_UINT8(chans[k].sev, g_sev);
    }
    static const struct {
        int ev;
        uint32_t value;
        const char *text;
        uint8_t sev;
    } ones[] = {
        {EV_ONE_HOLD_SENT, 17u, "TOF HOLD SENT BRAKE", EV_SEV_CRITICAL},
        {EV_ONE_HOLD_SENT, 5u, "TOF HOLD SENT LOITER", EV_SEV_CRITICAL},
        {EV_ONE_HOLD_REJECTED, 0u, "TOF HOLD REJECTED", EV_SEV_CRITICAL},
        {EV_ONE_REBOOTED, 0u, "TOF REBOOTED IN FLIGHT", EV_SEV_CRITICAL},
        {EV_ONE_PILOT_RESUMED, 0u, "TOF FAULT ACTIVE, AUTO WITHOUT HOLD", EV_SEV_WARNING},
        {EV_ONE_PARAMS_DEFAULT, 0u, "TOF PARAMS DEFAULT", EV_SEV_WARNING},
        {EV_ONE_ORIENT, 0u, "TOF ORIENT NOT CONFIRMED", EV_SEV_WARNING},
    };
    for (unsigned k = 0; k < sizeof ones / sizeof ones[0]; k++) {
        ev_raise(&g_e, (ev_oneshot_t)ones[k].ev, ones[k].value);
        t += 2000u;
        TEST_ASSERT_TRUE(next(t));
        TEST_ASSERT_EQUAL_STRING(ones[k].text, g_text);
        TEST_ASSERT_EQUAL_UINT8(ones[k].sev, g_sev);
    }
    ev_raise(&g_e, EV_ONE_READY, 0u);
    TEST_ASSERT_TRUE(next(t + 2000u));
    TEST_ASSERT_EQUAL_MEMORY("TOF READY ", g_text, 10u);
    TEST_ASSERT_NOT_NULL(strstr(g_text, tof_version_string()));
    TEST_ASSERT_NOT_NULL(strstr(g_text, tof_version_git_hash()));
    TEST_ASSERT_TRUE(strlen(g_text) <= EV_TEXT_LEN);
    TEST_ASSERT_FALSE(next(t + 4000u));
}

/* DIA-03: obstacle at WARN / CRIT with distance and azimuth; gone -> INFO. */
void test_DIA03_obstacle_messages(void)
{
    obst(PHASE_FLIGHT, 205u, NONE);
    TEST_ASSERT_TRUE(next(0u));
    TEST_ASSERT_EQUAL_STRING("TOF OBST WARN 2.05m 15deg", g_text);
    TEST_ASSERT_EQUAL_UINT8(EV_SEV_WARNING, g_sev);
    obst(PHASE_FLIGHT, 118u, NONE);
    TEST_ASSERT_TRUE(next(1000u));
    TEST_ASSERT_EQUAL_STRING("TOF OBST CRIT 1.18m 15deg", g_text);
    TEST_ASSERT_EQUAL_UINT8(EV_SEV_CRITICAL, g_sev);
    obst(PHASE_FLIGHT, NONE, NONE);
    TEST_ASSERT_TRUE(next(2000u));
    TEST_ASSERT_EQUAL_STRING("TOF OBST CLEAR", g_text);
    TEST_ASSERT_EQUAL_UINT8(EV_SEV_INFO, g_sev);
}

/* DIA-04: only on a change of state - the same state is never repeated. */
void test_DIA04_only_on_change(void)
{
    obst(PHASE_FLIGHT, 200u, NONE);
    TEST_ASSERT_TRUE(next(0u));
    for (uint32_t k = 1u; k < 20u; k++) {
        obst(PHASE_FLIGHT, (uint16_t)(200u - k), NONE);
        TEST_ASSERT_FALSE(next(k * 1000u));
    }
    /* A change that reverts before it could be sent is not reported at all. */
    obst(PHASE_FLIGHT, 100u, NONE);
    obst(PHASE_FLIGHT, 200u, NONE);
    TEST_ASSERT_FALSE(next(30000u));
}

/* DIA-04: WARN_HYST_CM hysteresis on both thresholds. */
void test_DIA04_hysteresis(void)
{
    obst(PHASE_FLIGHT, 250u, NONE);
    TEST_ASSERT_EQUAL_UINT8(1u, g_e.cur[EV_CH_OBSTACLE]);
    obst(PHASE_FLIGHT, 280u, NONE);
    TEST_ASSERT_EQUAL_UINT8(1u, g_e.cur[EV_CH_OBSTACLE]);
    obst(PHASE_FLIGHT, 281u, NONE);
    TEST_ASSERT_EQUAL_UINT8(0u, g_e.cur[EV_CH_OBSTACLE]);
    obst(PHASE_FLIGHT, 251u, NONE);
    TEST_ASSERT_EQUAL_UINT8(0u, g_e.cur[EV_CH_OBSTACLE]);
    obst(PHASE_FLIGHT, 120u, NONE);
    TEST_ASSERT_EQUAL_UINT8(2u, g_e.cur[EV_CH_OBSTACLE]);
    obst(PHASE_FLIGHT, 150u, NONE);
    TEST_ASSERT_EQUAL_UINT8(2u, g_e.cur[EV_CH_OBSTACLE]);
    obst(PHASE_FLIGHT, 151u, NONE);
    TEST_ASSERT_EQUAL_UINT8(1u, g_e.cur[EV_CH_OBSTACLE]);
    obst(PHASE_FLIGHT, 121u, NONE);
    TEST_ASSERT_EQUAL_UINT8(1u, g_e.cur[EV_CH_OBSTACLE]);
}

/* DIA-04: <= 1 message/s per type; a held-back change is sent when allowed. */
void test_DIA04_rate_per_type(void)
{
    obst(PHASE_FLIGHT, 200u, NONE);
    TEST_ASSERT_TRUE(next(0u)); /* WARN */
    obst(PHASE_FLIGHT, NONE, NONE);
    TEST_ASSERT_TRUE(next(100u)); /* CLEAR - another type */
    obst(PHASE_FLIGHT, 200u, NONE);
    TEST_ASSERT_FALSE(next(200u)); /* WARN again within 1 s */
    TEST_ASSERT_FALSE(next(999u));
    TEST_ASSERT_TRUE(next(1000u));
    TEST_ASSERT_EQUAL_MEMORY("TOF OBST WARN", g_text, 13u);
}

/* DIA-04: <= 3 messages/s in total; CRITICAL goes first. */
void test_DIA04_rate_total_and_priority(void)
{
    ev_raise(&g_e, EV_ONE_ORIENT, 0u);
    ev_raise(&g_e, EV_ONE_PARAMS_DEFAULT, 0u);
    ev_raise(&g_e, EV_ONE_READY, 0u);
    ev_set_state(&g_e, EV_CH_MAP_LOST, 1u);
    ev_set_state(&g_e, EV_CH_SENSOR_C, 1u);
    TEST_ASSERT_TRUE(next(0u));
    TEST_ASSERT_EQUAL_UINT8(EV_SEV_CRITICAL, g_sev);
    TEST_ASSERT_TRUE(next(0u));
    TEST_ASSERT_EQUAL_UINT8(EV_SEV_CRITICAL, g_sev);
    TEST_ASSERT_TRUE(next(10u));
    TEST_ASSERT_EQUAL_UINT8(EV_SEV_WARNING, g_sev);
    TEST_ASSERT_FALSE(next(500u));
    TEST_ASSERT_FALSE(next(999u));
    TEST_ASSERT_TRUE(next(1000u));
    TEST_ASSERT_EQUAL_UINT8(EV_SEV_WARNING, g_sev);
    TEST_ASSERT_TRUE(next(1000u));
    TEST_ASSERT_EQUAL_UINT8(EV_SEV_INFO, g_sev); /* READY last */
    TEST_ASSERT_FALSE(next(5000u));
    TEST_ASSERT_EQUAL_UINT32(5u, g_e.sent_total);
}

/* DIA-04: deduplication - an event raised several times before it is sent gives one message. */
void test_DIA04_dedup_pending(void)
{
    for (int k = 0; k < 5; k++) {
        ev_raise(&g_e, EV_ONE_HOLD_REJECTED, 0u);
    }
    TEST_ASSERT_TRUE(next(0u));
    TEST_ASSERT_FALSE(next(2000u));
}

/* DIA-04, MAP-17: in GROUND / ARMED_GROUND obstacle warnings come only from real obstacles, never
 * from GROUND_SUSPECT (the ground at 66 cm in the exported-with-ground distance). */
void test_DIA04_no_ground_suspect_warnings_on_ground(void)
{
    obst(PHASE_GROUND, 66u, NONE);
    TEST_ASSERT_FALSE(next(0u));
    obst(PHASE_ARMED_GROUND, 66u, NONE);
    TEST_ASSERT_FALSE(next(2000u));
    obst(PHASE_GROUND_RECOVER, 66u, NONE);
    TEST_ASSERT_FALSE(next(4000u));
    /* A real obstacle on the ground is warned about. */
    obst(PHASE_ARMED_GROUND, 66u, 200u);
    TEST_ASSERT_TRUE(next(6000u));
    TEST_ASSERT_EQUAL_STRING("TOF OBST WARN 2.00m -5deg", g_text);
    /* In the air the exported distance is used. */
    obst(PHASE_FLIGHT, 66u, 200u);
    TEST_ASSERT_TRUE(next(8000u));
    TEST_ASSERT_EQUAL_STRING("TOF OBST CRIT 0.66m 15deg", g_text);
}

/* DIA-03: sensor lost and restored; data stale / map lost / link / phase return silently. */
void test_DIA03_sensor_restored_and_silent_clears(void)
{
    ev_set_state(&g_e, EV_CH_SENSOR_R, 1u);
    TEST_ASSERT_TRUE(next(0u));
    ev_set_state(&g_e, EV_CH_SENSOR_R, 0u);
    TEST_ASSERT_TRUE(next(100u));
    TEST_ASSERT_EQUAL_STRING("TOF SENSOR R OK", g_text);
    TEST_ASSERT_EQUAL_UINT8(EV_SEV_INFO, g_sev);
    ev_set_state(&g_e, EV_CH_STALE, 1u);
    TEST_ASSERT_TRUE(next(200u));
    ev_set_state(&g_e, EV_CH_STALE, 0u);
    TEST_ASSERT_FALSE(next(2000u));
    ev_set_state(&g_e, EV_CH_STALE, 1u);
    TEST_ASSERT_TRUE(next(3000u));
    TEST_ASSERT_EQUAL_STRING("TOF DATA STALE", g_text);
    ev_set_state(&g_e, EV_CH_COUNT, 1u); /* ignored */
    ev_raise(&g_e, EV_ONE_COUNT, 0u);    /* ignored */
    TEST_ASSERT_FALSE(next(9000u));
}

/* MAP-13, DIA-06: log records in order; overflow is counted, never blocks. */
void test_DIA06_log_ring(void)
{
    ev_log_t r;
    TEST_ASSERT_FALSE(ev_take_log(&g_e, &r));
    ev_log(&g_e, 5u, EV_LOG_HOLD_EXPIRED, 1u, 70u);
    ev_log(&g_e, 6u, EV_LOG_COMMAND_ACK, 0u, 176u);
    TEST_ASSERT_TRUE(ev_take_log(&g_e, &r));
    TEST_ASSERT_EQUAL_UINT8(EV_LOG_HOLD_EXPIRED, r.kind);
    TEST_ASSERT_EQUAL_UINT8(1u, r.a);
    TEST_ASSERT_EQUAL_UINT16(70u, r.b);
    TEST_ASSERT_EQUAL_UINT32(5u, r.t_us);
    TEST_ASSERT_TRUE(ev_take_log(&g_e, &r));
    TEST_ASSERT_EQUAL_UINT8(EV_LOG_COMMAND_ACK, r.kind);
    for (uint32_t k = 0; k < EV_LOG_LEN + 3u; k++) {
        ev_log(&g_e, k, EV_LOG_PHASE, 0u, 0u);
    }
    TEST_ASSERT_EQUAL_UINT32(3u, g_e.log_dropped);
    uint32_t n = 0u;
    while (ev_take_log(&g_e, &r)) {
        TEST_ASSERT_EQUAL_UINT32(n, r.t_us);
        n++;
    }
    TEST_ASSERT_EQUAL_UINT32(EV_LOG_LEN, n);
    /* A sent STATUSTEXT is logged too. */
    ev_raise(&g_e, EV_ONE_ORIENT, 0u);
    TEST_ASSERT_TRUE(next(0u));
    TEST_ASSERT_TRUE(ev_take_log(&g_e, &r));
    TEST_ASSERT_EQUAL_UINT8(EV_LOG_STATUSTEXT, r.kind);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_DIA05_thresholds);
    RUN_TEST(test_DIA03_severity_values);
    RUN_TEST(test_DIA03_all_texts);
    RUN_TEST(test_DIA03_obstacle_messages);
    RUN_TEST(test_DIA04_only_on_change);
    RUN_TEST(test_DIA04_hysteresis);
    RUN_TEST(test_DIA04_rate_per_type);
    RUN_TEST(test_DIA04_rate_total_and_priority);
    RUN_TEST(test_DIA04_dedup_pending);
    RUN_TEST(test_DIA04_no_ground_suspect_warnings_on_ground);
    RUN_TEST(test_DIA03_sensor_restored_and_silent_clears);
    RUN_TEST(test_DIA06_log_ring);
    return UNITY_END();
}
