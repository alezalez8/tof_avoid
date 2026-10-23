#include <string.h>

#include "core/tof_core.h"
#include "unity.h"

#define MS 1000u
#define PERIOD_US 66666u

static params_t g_p;
static tof_core_t g_c;
static tof_frame_t g_fr;

void setUp(void)
{
    params_set_defaults(&g_p);
    tof_core_init(&g_c, &g_p, 0u);
}

void tearDown(void) {}

static void make_frame(uint8_t s, uint32_t no, uint32_t t, int16_t obstacle_mm)
{
    memset(&g_fr, 0, sizeof g_fr);
    g_fr.sensor_id = s;
    g_fr.frame_no = no;
    g_fr.t_ready_us = t;
    g_fr.res = 4u;
    g_fr.freq_hz = 15u;
    if (obstacle_mm > 0) {
        tof_zone_t *z = &g_fr.zone[1u * 4u + 2u];
        z->nb_target_detected = 1u;
        z->target[0].distance_mm = obstacle_mm;
        z->target[0].target_status = 5u;
    }
}

/* Runs `n` frame periods; sensors in `alive_mask` deliver frames. Returns maps produced. */
static uint32_t run(tof_core_t *c, uint32_t t0, uint32_t n, uint8_t alive_mask, int16_t c_obstacle_mm)
{
    uint32_t maps = 0u;
    for (uint32_t k = 0; k < n; k++) {
        const uint32_t t = t0 + k * PERIOD_US;
        for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
            if (((alive_mask >> s) & 1u) != 0u) {
                make_frame(s, k, t, (s == TOF_SENSOR_C) ? c_obstacle_mm : 0);
                TEST_ASSERT_EQUAL_INT(TOF_PUSH_OK, tof_core_push_frame(c, &g_fr, t));
            }
        }
        maps += tof_core_update(c, t) ? 1u : 0u;
        /* Poll between frames as the main loop would. */
        maps += tof_core_update(c, t + PERIOD_US / 2u) ? 1u : 0u;
    }
    return maps;
}

/* SNS-09: frame metadata reaches the core (sensor id, number, MCU time, mode, frequency). */
void test_SNS09_frame_metadata_is_used(void)
{
    make_frame(TOF_SENSOR_R, 41u, 5000u, 0);
    TEST_ASSERT_EQUAL_INT(TOF_PUSH_OK, tof_core_push_frame(&g_c, &g_fr, 5100u));
    TEST_ASSERT_TRUE(g_c.fusion.frame[TOF_SENSOR_R].fresh);
    TEST_ASSERT_EQUAL_UINT32(5000u, g_c.fusion.frame[TOF_SENSOR_R].t_us);
    TEST_ASSERT_EQUAL_UINT32(41u, g_c.fusion.frame[TOF_SENSOR_R].frame_no);
    TEST_ASSERT_EQUAL_UINT32(5100u, g_c.sh[TOF_SENSOR_R].t_last_frame_us);
    TEST_ASSERT_EQUAL_UINT32(1u, g_c.stats[TOF_SENSOR_R].frames);
    /* Frame number gaps are counted. */
    make_frame(TOF_SENSOR_R, 42u, 6000u, 0);
    (void)tof_core_push_frame(&g_c, &g_fr, 6000u);
    make_frame(TOF_SENSOR_R, 45u, 7000u, 0);
    (void)tof_core_push_frame(&g_c, &g_fr, 7000u);
    TEST_ASSERT_EQUAL_UINT32(1u, g_c.stats[TOF_SENSOR_R].gaps);
}

void test_SNS09_bad_sensor_id_ignored(void)
{
    make_frame(7u, 1u, 0u, 0);
    TEST_ASSERT_EQUAL_INT(TOF_PUSH_BAD_SENSOR, tof_core_push_frame(&g_c, &g_fr, 0u));
    tof_core_sensor_error(&g_c, 7u, 0u);
    TEST_ASSERT_FALSE(tof_core_take_reinit_request(&g_c, 7u));
    tof_core_reinit_done(&g_c, 7u, true, 0u);
    TEST_ASSERT_EQUAL_INT(SENSOR_FAILED, tof_core_sensor_health(&g_c, 7u));
}

/* A frame in the wrong mode is rejected and counts as a sensor error (SNS-11). */
void test_SNS09_wrong_mode_is_error(void)
{
    for (uint32_t k = 0; k < 3u; k++) {
        make_frame(TOF_SENSOR_L, k, k * MS, 0);
        g_fr.freq_hz = (k == 0u) ? 30u : 15u;
        g_fr.res = (k == 0u) ? 4u : 8u;
        TEST_ASSERT_EQUAL_INT(TOF_PUSH_BAD_MODE, tof_core_push_frame(&g_c, &g_fr, k * MS));
    }
    TEST_ASSERT_EQUAL_UINT32(3u, g_c.stats[TOF_SENSOR_L].rejected);
    TEST_ASSERT_TRUE(tof_core_take_reinit_request(&g_c, TOF_SENSOR_L));
    TEST_ASSERT_FALSE(tof_core_take_reinit_request(&g_c, TOF_SENSOR_C));
}

/* End to end: three sensors at 15 Hz give one map per frame period with the obstacle. */
void test_MAP09_core_one_map_per_frame_period(void)
{
    const uint32_t maps = run(&g_c, 0u, 10u, 0x7u, 2000);
    TEST_ASSERT_EQUAL_UINT32(10u, maps);
    uint16_t d[TOF_NUM_SECTORS];
    tof_core_export(&g_c, 9u * PERIOD_US, d);
    TEST_ASSERT_UINT16_WITHIN(1u, 199u, d[1]);
    TEST_ASSERT_EQUAL_UINT16(FUSION_DIST_UNKNOWN, d[30]);
    TEST_ASSERT_EQUAL_INT(MAP_OK, tof_core_map_state(&g_c));
}

/* SNS-11 / MAP-10 / MAP-11: one sensor stops; only it is reinitialised, the map goes DEGRADED,
 * the other two keep producing maps (fused after the window). */
void test_SNS11_core_one_sensor_stops(void)
{
    (void)run(&g_c, 0u, 5u, 0x7u, 2000);
    const uint32_t t1 = 5u * PERIOD_US;
    const uint32_t maps = run(&g_c, t1, 10u, 0x5u, 0); /* C dead */
    TEST_ASSERT_TRUE(maps >= 9u); /* the last pair is still pending */
    TEST_ASSERT_EQUAL_UINT32(0u, g_c.fusion.frames_late_total);
    TEST_ASSERT_TRUE(tof_core_take_reinit_request(&g_c, TOF_SENSOR_C));
    TEST_ASSERT_FALSE(tof_core_take_reinit_request(&g_c, TOF_SENSOR_L));
    TEST_ASSERT_FALSE(tof_core_take_reinit_request(&g_c, TOF_SENSOR_R));
    TEST_ASSERT_EQUAL_INT(SENSOR_DEGRADED, tof_core_sensor_health(&g_c, TOF_SENSOR_C));
    TEST_ASSERT_EQUAL_INT(SENSOR_OK, tof_core_sensor_health(&g_c, TOF_SENSOR_L));
    TEST_ASSERT_EQUAL_INT(MAP_DEGRADED, tof_core_map_state(&g_c));

    /* Reinit succeeds and frames come back: OK again. */
    const uint32_t t2 = t1 + 10u * PERIOD_US;
    tof_core_reinit_done(&g_c, TOF_SENSOR_C, true, t2);
    (void)run(&g_c, t2, 3u, 0x7u, 0);
    TEST_ASSERT_EQUAL_INT(SENSOR_OK, tof_core_sensor_health(&g_c, TOF_SENSOR_C));
    TEST_ASSERT_EQUAL_INT(MAP_OK, tof_core_map_state(&g_c));
}

/* MAP-11: all sensors silent -> LOST. */
void test_MAP11_core_all_sensors_silent_lost(void)
{
    (void)run(&g_c, 0u, 3u, 0x7u, 0);
    const uint32_t t1 = 3u * PERIOD_US;
    TEST_ASSERT_FALSE(tof_core_update(&g_c, t1 + 100u * MS));
    TEST_ASSERT_FALSE(tof_core_update(&g_c, t1 + 400u * MS));
    TEST_ASSERT_EQUAL_INT(MAP_LOST, tof_core_map_state(&g_c));
    uint16_t d[TOF_NUM_SECTORS];
    tof_core_export(&g_c, t1 + 400u * MS, d);
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        TEST_ASSERT_EQUAL_UINT16(FUSION_DIST_UNKNOWN, d[k]); /* STALE everywhere */
    }
}

/* SW-02 / 5: the same input gives the same output. */
void test_MAP05_core_is_deterministic(void)
{
    static tof_core_t a;
    static tof_core_t b;
    tof_core_init(&a, &g_p, 0u);
    tof_core_init(&b, &g_p, 0u);
    (void)run(&a, 0u, 7u, 0x7u, 1234);
    (void)run(&b, 0u, 7u, 0x7u, 1234);
    uint16_t da[TOF_NUM_SECTORS];
    uint16_t db[TOF_NUM_SECTORS];
    tof_core_export(&a, 7u * PERIOD_US, da);
    tof_core_export(&b, 7u * PERIOD_US, db);
    TEST_ASSERT_EQUAL_MEMORY(da, db, sizeof da);
    TEST_ASSERT_EQUAL_MEMORY(a.fusion.cell, b.fusion.cell, sizeof a.fusion.cell);
}

/* MAP-12: a sensor in reinit is not waited for: the map follows the healthy sensors at once. */
void test_MAP09_core_does_not_wait_for_reinit_sensor(void)
{
    (void)run(&g_c, 0u, 2u, 0x7u, 0);
    const uint32_t t = 2u * PERIOD_US;
    tof_core_sensor_error(&g_c, TOF_SENSOR_C, t);
    tof_core_sensor_error(&g_c, TOF_SENSOR_C, t);
    tof_core_sensor_error(&g_c, TOF_SENSOR_C, t);
    TEST_ASSERT_TRUE(tof_core_take_reinit_request(&g_c, TOF_SENSOR_C));
    make_frame(TOF_SENSOR_L, 2u, t, 0);
    (void)tof_core_push_frame(&g_c, &g_fr, t);
    make_frame(TOF_SENSOR_R, 2u, t, 0);
    (void)tof_core_push_frame(&g_c, &g_fr, t);
    TEST_ASSERT_TRUE(tof_core_update(&g_c, t));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_SNS09_frame_metadata_is_used);
    RUN_TEST(test_SNS09_bad_sensor_id_ignored);
    RUN_TEST(test_SNS09_wrong_mode_is_error);
    RUN_TEST(test_MAP09_core_one_map_per_frame_period);
    RUN_TEST(test_SNS11_core_one_sensor_stops);
    RUN_TEST(test_MAP11_core_all_sensors_silent_lost);
    RUN_TEST(test_MAP05_core_is_deterministic);
    RUN_TEST(test_MAP09_core_does_not_wait_for_reinit_sensor);
    return UNITY_END();
}
