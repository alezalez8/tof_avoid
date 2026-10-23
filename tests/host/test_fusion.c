#include <math.h>
#include <string.h>

#include "core/fusion.h"
#include "unity.h"

#define MS 1000u
#define T0 (1000u * MS)

static params_t g_p;
static fusion_t g_f;
static tof_frame_t g_fr[TOF_NUM_SENSORS];
static const sensor_health_t k_all_ok[TOF_NUM_SENSORS] = {SENSOR_OK, SENSOR_OK, SENSOR_OK};

void setUp(void)
{
    params_set_defaults(&g_p);
    fusion_init(&g_f, &g_p);
}

void tearDown(void) {}

static void enable_clear(void)
{
    (void)params_set_u32(&g_p, PARAM_AMBIENT_CLEAR_MAX, 1000u);
    fusion_init(&g_f, &g_p);
}

/* Frame with no detection in any zone. */
static void frame_empty(uint8_t s, uint32_t t)
{
    memset(&g_fr[s], 0, sizeof g_fr[s]);
    g_fr[s].sensor_id = s;
    g_fr[s].t_ready_us = t;
    g_fr[s].res = 4u;
    g_fr[s].freq_hz = 15u;
}

static void zone_target(uint8_t s, uint8_t i, uint8_t j, int16_t mm, uint8_t status)
{
    tof_zone_t *z = &g_fr[s].zone[i * 4u + j]; /* identity ORIENT_TRANSFORM by default */
    z->nb_target_detected = 1u;
    z->target[0].distance_mm = mm;
    z->target[0].target_status = status;
    z->target[0].range_sigma_mm = 5u;
    z->target[0].signal_per_spad = 100u;
}

static void push(uint8_t s)
{
    TEST_ASSERT_TRUE(fusion_push_frame(&g_f, &g_fr[s]));
}

static void push_empty_all(uint32_t t)
{
    for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
        frame_empty(s, t);
        push(s);
    }
}

static uint16_t expect_cm(uint8_t i, float r_m)
{
    return (uint16_t)(r_m * cosf(geo_deg_to_rad(geo_zone_beta_deg(4u, i))) * 100.0f);
}

static void assert_sectors(uint8_t first, uint8_t last, export_state_t st, uint32_t now)
{
    for (uint8_t k = first; k <= last; k++) {
        TEST_ASSERT_EQUAL_INT_MESSAGE(st, fusion_export_state(&g_f, k, now), "sector");
    }
}

/* ---- Configuration ---- */

void test_MAP09_window_from_params(void)
{
    TEST_ASSERT_UINT32_WITHIN(2u, 100u * MS, g_f.window_us); /* 1.5 periods at 15 Hz */
    (void)params_set_u32(&g_p, PARAM_SENSOR_FREQ_HZ, 10u);
    fusion_init(&g_f, &g_p);
    TEST_ASSERT_UINT32_WITHIN(2u, 150u * MS, g_f.window_us);
    (void)params_set_u32(&g_p, PARAM_FUSION_WINDOW_MS, 50u);
    fusion_init(&g_f, &g_p);
    TEST_ASSERT_EQUAL_UINT32(50u * MS, g_f.window_us);
    TEST_ASSERT_EQUAL_UINT32(150u * MS, g_f.max_age_us);
}

/* Default mounts -30/0/+30, 4x4, CAL_TOL 2: FoV is -54.5..54.5 deg -> sectors 61..71 and 0..11. */
void test_MAP05_in_fov_sectors_default_mounts(void)
{
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        const bool expect = (k <= 11u) || (k >= 61u);
        TEST_ASSERT_EQUAL_MESSAGE(expect, g_f.in_fov[k], "sector");
    }
}

void test_MAP05_out_of_fov_exports_65535(void)
{
    uint16_t d[TOF_NUM_SECTORS];
    push_empty_all(T0);
    fusion_run(&g_f, k_all_ok, T0);
    fusion_export(&g_f, T0, d);
    TEST_ASSERT_EQUAL_INT(EXPORT_OUT_OF_FOV, fusion_export_state(&g_f, 30u, T0));
    TEST_ASSERT_EQUAL_UINT16(FUSION_DIST_UNKNOWN, d[30]);
    TEST_ASSERT_EQUAL_UINT16(FUSION_DIST_UNKNOWN, d[36]);
}

void test_MAP05_push_rejects_wrong_resolution_or_sensor(void)
{
    frame_empty(TOF_SENSOR_C, T0);
    g_fr[TOF_SENSOR_C].res = 8u;
    TEST_ASSERT_FALSE(fusion_push_frame(&g_f, &g_fr[TOF_SENSOR_C]));
    frame_empty(TOF_SENSOR_C, T0);
    g_fr[TOF_SENSOR_C].sensor_id = 3u;
    TEST_ASSERT_FALSE(fusion_push_frame(&g_f, &g_fr[TOF_SENSOR_C]));
}

/* ---- MAP-05: extent + CAL_TOL, no interpolation ---- */

/* C zone column 2 spans 0..11.25 deg, +-2 -> -2..13.25: sectors 0, 1, 2, 3, the same distance. */
void test_MAP05_zone_covers_all_sectors_of_extent_without_interpolation(void)
{
    frame_empty(TOF_SENSOR_C, T0);
    zone_target(TOF_SENSOR_C, 1u, 2u, 2000, 5u);
    push(TOF_SENSOR_C);
    fusion_run(&g_f, k_all_ok, T0);
    uint16_t d[TOF_NUM_SECTORS];
    fusion_export(&g_f, T0, d);
    const uint16_t e = expect_cm(1u, 2.0f);
    for (uint8_t k = 0; k <= 3u; k++) {
        TEST_ASSERT_EQUAL_INT(EXPORT_OCCUPIED, fusion_export_state(&g_f, k, T0));
        TEST_ASSERT_UINT16_WITHIN(1u, e, d[k]);
        TEST_ASSERT_EQUAL_UINT16(d[0], d[k]); /* no interpolation between sectors */
    }
    TEST_ASSERT_EQUAL_UINT16(FUSION_DIST_UNKNOWN, d[4]);
    TEST_ASSERT_EQUAL_UINT16(FUSION_DIST_UNKNOWN, d[71]);
    /* Source and angular reliability are kept in the cell. */
    TEST_ASSERT_EQUAL_UINT8(TOF_SENSOR_C, g_f.cell[1].src_sensor);
    TEST_ASSERT_EQUAL_UINT8(1u * 4u + 2u, g_f.cell[1].src_zone);
    TEST_ASSERT_TRUE(g_f.cell[1].angle_ok);
    TEST_ASSERT_FALSE(g_f.cell[3].angle_ok); /* reached only through CAL_TOL_DEG */
}

void test_MAP05_cal_tol_zero_narrows_coverage(void)
{
    (void)params_set_f32(&g_p, PARAM_CAL_TOL_DEG, 0.0f);
    fusion_init(&g_f, &g_p);
    frame_empty(TOF_SENSOR_C, T0);
    zone_target(TOF_SENSOR_C, 1u, 2u, 2000, 5u);
    push(TOF_SENSOR_C);
    fusion_run(&g_f, k_all_ok, T0);
    assert_sectors(0u, 2u, EXPORT_OCCUPIED, T0);
    TEST_ASSERT_EQUAL_INT(EXPORT_UNKNOWN, fusion_export_state(&g_f, 3u, T0));
}

/* Overlap: the nearest valid target wins. C col 3 -> sectors 2..5, R col 0 -> sectors 1..4. */
void test_MAP05_overlap_nearest_valid_target_wins(void)
{
    frame_empty(TOF_SENSOR_C, T0);
    frame_empty(TOF_SENSOR_R, T0);
    zone_target(TOF_SENSOR_C, 1u, 3u, 2000, 5u);
    zone_target(TOF_SENSOR_R, 1u, 0u, 1500, 9u);
    push(TOF_SENSOR_C);
    push(TOF_SENSOR_R);
    fusion_run(&g_f, k_all_ok, T0);
    uint16_t d[TOF_NUM_SECTORS];
    fusion_export(&g_f, T0, d);
    const uint16_t near = expect_cm(1u, 1.5f);
    const uint16_t far = expect_cm(1u, 2.0f);
    for (uint8_t k = 1; k <= 4u; k++) {
        TEST_ASSERT_UINT16_WITHIN(1u, near, d[k]);
    }
    TEST_ASSERT_UINT16_WITHIN(1u, far, d[5]);
    TEST_ASSERT_EQUAL_UINT8(TOF_SENSOR_R, g_f.cell[3].src_sensor);
}

/* An invalid zone of one sensor does not cancel the obstacle of another. */
void test_MAP05_invalid_zone_does_not_cancel_other_sensor(void)
{
    frame_empty(TOF_SENSOR_C, T0);
    frame_empty(TOF_SENSOR_R, T0);
    zone_target(TOF_SENSOR_C, 1u, 3u, 2000, 5u);
    zone_target(TOF_SENSOR_R, 1u, 0u, 1000, 4u); /* status 4: invalid */
    push(TOF_SENSOR_C);
    push(TOF_SENSOR_R);
    fusion_run(&g_f, k_all_ok, T0);
    assert_sectors(2u, 5u, EXPORT_OCCUPIED, T0);
    TEST_ASSERT_EQUAL_INT(EXPORT_UNKNOWN, fusion_export_state(&g_f, 1u, T0));
}

/* CLR requires every zone of the sector to agree. */
void test_MAP05_clr_requires_all_zones(void)
{
    enable_clear();
    for (uint32_t n = 0; n < 3u; n++) {
        const uint32_t t = T0 + n * 66u * MS;
        for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
            frame_empty(s, t);
        }
        zone_target(TOF_SENSOR_C, 1u, 2u, 1000, 4u); /* one invalid zone: sectors 0..3 */
        for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
            push(s);
        }
        fusion_run(&g_f, k_all_ok, t);
    }
    const uint32_t now = T0 + 132u * MS;
    assert_sectors(0u, 3u, EXPORT_UNKNOWN, now);
    assert_sectors(4u, 11u, EXPORT_CLEAR_MEASURED, now);
    assert_sectors(61u, 71u, EXPORT_CLEAR_MEASURED, now);
    uint16_t d[TOF_NUM_SECTORS];
    fusion_export(&g_f, now, d);
    TEST_ASSERT_EQUAL_UINT16(301u, d[71]); /* max_distance + 1 */
    TEST_ASSERT_EQUAL_UINT16(FUSION_DIST_UNKNOWN, d[0]);
}

/* A valid target beyond max_distance allows CLR; one below min_distance makes the zone INV. */
void test_MAP05_range_limits_beyond_max_clr_below_min_inv(void)
{
    enable_clear();
    for (uint32_t n = 0; n < 3u; n++) {
        const uint32_t t = T0 + n * 66u * MS;
        push_empty_all(t);
        zone_target(TOF_SENSOR_C, 1u, 1u, 3500, 5u); /* ~348 cm > 300 */
        push(TOF_SENSOR_C);
        fusion_run(&g_f, k_all_ok, t);
    }
    assert_sectors(0u, 11u, EXPORT_CLEAR_MEASURED, T0 + 132u * MS);
    assert_sectors(61u, 71u, EXPORT_CLEAR_MEASURED, T0 + 132u * MS);

    const uint32_t t = T0 + 200u * MS;
    push_empty_all(t);
    zone_target(TOF_SENSOR_C, 1u, 1u, 30, 5u); /* ~2 cm < MIN_DIST_CM 4 */
    push(TOF_SENSOR_C);
    fusion_run(&g_f, k_all_ok, t);
    /* C column 1 spans -11.25..0, +-2 -> sectors 69..71 and 0. */
    assert_sectors(69u, 71u, EXPORT_UNKNOWN, t);
    TEST_ASSERT_EQUAL_INT(EXPORT_UNKNOWN, fusion_export_state(&g_f, 0u, t));
    assert_sectors(1u, 11u, EXPORT_CLEAR_MEASURED, t);
    assert_sectors(61u, 68u, EXPORT_CLEAR_MEASURED, t);
}

/* Ambient above AMBIENT_CLEAR_MAX: an empty zone is not a confirmed absence (DEV-02). */
void test_MAP05_high_ambient_blocks_clr(void)
{
    enable_clear();
    for (uint32_t n = 0; n < 3u; n++) {
        const uint32_t t = T0 + n * 66u * MS;
        for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
            frame_empty(s, t);
        }
        g_fr[TOF_SENSOR_C].zone[1u * 4u + 2u].ambient_per_spad = 1001u;
        for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
            push(s);
        }
        fusion_run(&g_f, k_all_ok, t);
    }
    assert_sectors(0u, 3u, EXPORT_UNKNOWN, T0 + 132u * MS);
    assert_sectors(4u, 11u, EXPORT_CLEAR_MEASURED, T0 + 132u * MS);
}

/* With CLEAR_MEASURED off (default) empty zones are INV: nothing is ever cleared. */
void test_MAP05_clear_disabled_empty_zones_stay_unknown(void)
{
    for (uint32_t n = 0; n < 5u; n++) {
        push_empty_all(T0 + n * 66u * MS);
        fusion_run(&g_f, k_all_ok, T0 + n * 66u * MS);
    }
    assert_sectors(0u, 11u, EXPORT_UNKNOWN, T0 + 264u * MS);
    assert_sectors(61u, 71u, EXPORT_UNKNOWN, T0 + 264u * MS);
}

/* ---- MAP-04: horizontal distance, ROW_MASK ---- */

void test_MAP04_horizontal_distance_of_elevated_zone(void)
{
    frame_empty(TOF_SENSOR_C, T0);
    zone_target(TOF_SENSOR_C, 0u, 1u, 2000, 5u); /* top row: beta = 16.875 deg */
    push(TOF_SENSOR_C);
    fusion_run(&g_f, k_all_ok, T0);
    uint16_t d[TOF_NUM_SECTORS];
    fusion_export(&g_f, T0, d);
    TEST_ASSERT_UINT16_WITHIN(1u, 191u, d[71]); /* 2 m * cos(16.875) = 1.914 m */
}

/* Mount offset t_m enters the horizontal distance. */
void test_MAP04_mount_offset_in_distance(void)
{
    (void)params_set_f32(&g_p, PARAM_MOUNT_X_M_C, 0.10f);
    fusion_init(&g_f, &g_p);
    frame_empty(TOF_SENSOR_C, T0);
    zone_target(TOF_SENSOR_C, 1u, 2u, 2000, 5u);
    push(TOF_SENSOR_C);
    fusion_run(&g_f, k_all_ok, T0);
    uint16_t d[TOF_NUM_SECTORS];
    fusion_export(&g_f, T0, d);
    const float a = geo_deg_to_rad(5.625f);
    const float x = 2.0f * cosf(a) * cosf(a) + 0.10f;
    const float y = 2.0f * cosf(a) * sinf(a);
    TEST_ASSERT_UINT16_WITHIN(1u, (uint16_t)(sqrtf(x * x + y * y) * 100.0f), d[1]);
}

void test_MAP04_row_mask_excludes_rows(void)
{
    (void)params_set_u32(&g_p, PARAM_ROW_MASK, 0x07u); /* rows 0..2, bottom row 3 excluded */
    (void)params_set_u32(&g_p, PARAM_AMBIENT_CLEAR_MAX, 1000u);
    fusion_init(&g_f, &g_p);
    for (uint32_t n = 0; n < 3u; n++) {
        const uint32_t t = T0 + n * 66u * MS;
        push_empty_all(t);
        zone_target(TOF_SENSOR_C, 3u, 2u, 700, 5u); /* only in the masked row */
        push(TOF_SENSOR_C);
        fusion_run(&g_f, k_all_ok, t);
    }
    /* Masked zone neither sets an obstacle nor blocks CLR. */
    assert_sectors(0u, 11u, EXPORT_CLEAR_MEASURED, T0 + 132u * MS);
}

/* ---- MAP-09: fusion window, cell time = oldest contribution ---- */

void test_MAP09_frames_within_window_fused_with_oldest_time(void)
{
    frame_empty(TOF_SENSOR_L, T0);
    frame_empty(TOF_SENSOR_C, T0 + 30u * MS);
    frame_empty(TOF_SENSOR_R, T0 + 60u * MS);
    zone_target(TOF_SENSOR_L, 1u, 3u, 1000, 5u);
    zone_target(TOF_SENSOR_C, 1u, 3u, 1000, 5u);
    zone_target(TOF_SENSOR_R, 1u, 3u, 1000, 5u);
    push(TOF_SENSOR_L);
    TEST_ASSERT_FALSE(fusion_ready(&g_f, k_all_ok, T0 + 10u * MS));
    push(TOF_SENSOR_C);
    TEST_ASSERT_FALSE(fusion_ready(&g_f, k_all_ok, T0 + 40u * MS));
    push(TOF_SENSOR_R);
    TEST_ASSERT_TRUE(fusion_ready(&g_f, k_all_ok, T0 + 60u * MS));
    fusion_run(&g_f, k_all_ok, T0 + 60u * MS);
    TEST_ASSERT_EQUAL_UINT32(1u, g_f.map_seq);
    TEST_ASSERT_FALSE(fusion_ready(&g_f, k_all_ok, T0 + 61u * MS));
    /* Sector 70 is seen by L and C: time of L. Sector 3 by C and R: time of C. Sector 10: R only. */
    TEST_ASSERT_EQUAL_UINT32(T0, g_f.cell_t_us[70]);
    TEST_ASSERT_EQUAL_UINT32(T0 + 30u * MS, g_f.cell_t_us[3]);
    TEST_ASSERT_EQUAL_UINT32(T0 + 60u * MS, g_f.cell_t_us[10]);
    /* All three obstacles were used in one step. */
    TEST_ASSERT_EQUAL_INT(CELL_OCCUPIED, g_f.cell[69].state); /* L col 3: -20.75..-5.5 deg */
    TEST_ASSERT_EQUAL_INT(CELL_OCCUPIED, g_f.cell[3].state);
    TEST_ASSERT_EQUAL_INT(CELL_OCCUPIED, g_f.cell[9].state);
}

void test_MAP09_ready_after_window_without_all_sensors(void)
{
    frame_empty(TOF_SENSOR_C, T0);
    push(TOF_SENSOR_C);
    TEST_ASSERT_FALSE(fusion_ready(&g_f, k_all_ok, T0 + g_f.window_us - 1u));
    TEST_ASSERT_TRUE(fusion_ready(&g_f, k_all_ok, T0 + g_f.window_us));
    /* FAILED sensors are not waited for. */
    const sensor_health_t h[TOF_NUM_SENSORS] = {SENSOR_FAILED, SENSOR_OK, SENSOR_FAILED};
    TEST_ASSERT_TRUE(fusion_ready(&g_f, h, T0));
    /* Nothing fresh: never ready. */
    fusion_run(&g_f, k_all_ok, T0);
    TEST_ASSERT_FALSE(fusion_ready(&g_f, k_all_ok, T0 + 10000u * MS));
}

void test_MAP09_frame_older_than_window_is_dropped(void)
{
    frame_empty(TOF_SENSOR_L, T0);
    zone_target(TOF_SENSOR_L, 1u, 1u, 1000, 5u);
    frame_empty(TOF_SENSOR_C, T0 + 150u * MS);
    push(TOF_SENSOR_L);
    push(TOF_SENSOR_C);
    fusion_run(&g_f, k_all_ok, T0 + 150u * MS);
    TEST_ASSERT_EQUAL_UINT32(1u, g_f.frames_late_total);
    TEST_ASSERT_FALSE(g_f.frame[TOF_SENSOR_L].fresh);
    for (uint8_t k = 61u; k <= 71u; k++) {
        TEST_ASSERT_NOT_EQUAL(CELL_OCCUPIED, g_f.cell[k].state);
    }
}

/* Data older than VALID_DATA_MAX_AGE_MS is STALE and exported as 65535. */
void test_MAP09_stale_after_max_age(void)
{
    frame_empty(TOF_SENSOR_C, T0);
    zone_target(TOF_SENSOR_C, 1u, 2u, 2000, 5u);
    push(TOF_SENSOR_C);
    fusion_run(&g_f, k_all_ok, T0);
    uint16_t d[TOF_NUM_SECTORS];
    fusion_export(&g_f, T0 + 150u * MS, d);
    TEST_ASSERT_NOT_EQUAL(FUSION_DIST_UNKNOWN, d[1]);
    TEST_ASSERT_EQUAL_INT(EXPORT_STALE, fusion_export_state(&g_f, 1u, T0 + 150u * MS + 1u));
    fusion_export(&g_f, T0 + 150u * MS + 1u, d);
    TEST_ASSERT_EQUAL_UINT16(FUSION_DIST_UNKNOWN, d[1]);
    /* A sector that never had data is STALE too. */
    fusion_init(&g_f, &g_p);
    TEST_ASSERT_EQUAL_INT(EXPORT_STALE, fusion_export_state(&g_f, 0u, T0));
}

/* ---- MAP-10: zones of a FAILED sensor are INV ---- */

void test_MAP10_failed_sensor_zones_are_inv(void)
{
    const sensor_health_t h[TOF_NUM_SENSORS] = {SENSOR_OK, SENSOR_FAILED, SENSOR_OK};
    frame_empty(TOF_SENSOR_C, T0);
    zone_target(TOF_SENSOR_C, 1u, 2u, 2000, 5u);
    push(TOF_SENSOR_C);
    fusion_run(&g_f, h, T0);
    for (uint8_t k = 0; k <= 3u; k++) {
        TEST_ASSERT_EQUAL_INT(CELL_UNKNOWN, g_f.cell[k].state);
    }
}

/* DEGRADED sensor data is processed normally (DEV-06 discussion: only FAILED is "faulty"). */
void test_MAP10_degraded_sensor_still_reports_obstacles(void)
{
    const sensor_health_t h[TOF_NUM_SENSORS] = {SENSOR_OK, SENSOR_DEGRADED, SENSOR_OK};
    frame_empty(TOF_SENSOR_C, T0);
    zone_target(TOF_SENSOR_C, 1u, 2u, 2000, 5u);
    push(TOF_SENSOR_C);
    fusion_run(&g_f, h, T0);
    TEST_ASSERT_EQUAL_INT(CELL_OCCUPIED, g_f.cell[1].state);
}

void test_MAP10_valid_fraction_of_frame(void)
{
    frame_empty(TOF_SENSOR_C, T0);
    for (uint8_t j = 0; j < 4u; j++) {
        zone_target(TOF_SENSOR_C, 0u, j, 1000, 255u); /* 4 of 16 zones invalid */
    }
    push(TOF_SENSOR_C);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.75f, g_f.frame[TOF_SENSOR_C].valid_frac);
}

/* ---- MAP-11: map state ---- */

void test_MAP11_map_state_ok_degraded_lost(void)
{
    const sensor_health_t ok[3] = {SENSOR_OK, SENSOR_OK, SENSOR_OK};
    const sensor_health_t one_deg[3] = {SENSOR_OK, SENSOR_DEGRADED, SENSOR_OK};
    const sensor_health_t one_fail[3] = {SENSOR_FAILED, SENSOR_OK, SENSOR_OK};
    const sensor_health_t two_bad[3] = {SENSOR_FAILED, SENSOR_OK, SENSOR_DEGRADED};
    const sensor_health_t none_ok[3] = {SENSOR_FAILED, SENSOR_DEGRADED, SENSOR_DEGRADED};
    TEST_ASSERT_EQUAL_INT(MAP_OK, fusion_map_state(ok));
    TEST_ASSERT_EQUAL_INT(MAP_DEGRADED, fusion_map_state(one_deg));
    TEST_ASSERT_EQUAL_INT(MAP_DEGRADED, fusion_map_state(one_fail));
    TEST_ASSERT_EQUAL_INT(MAP_DEGRADED, fusion_map_state(two_bad));
    TEST_ASSERT_EQUAL_INT(MAP_LOST, fusion_map_state(none_ok));
    push_empty_all(T0);
    fusion_run(&g_f, none_ok, T0);
    TEST_ASSERT_EQUAL_INT(MAP_LOST, g_f.state);
}

/* ---- MAP-13 inside the map: invalid data holds the obstacle until HOLD_MAX_MS ---- */

void test_MAP13_map_holds_obstacle_on_invalid_frames_then_unknown(void)
{
    frame_empty(TOF_SENSOR_C, T0);
    zone_target(TOF_SENSOR_C, 1u, 2u, 2000, 5u);
    push(TOF_SENSOR_C);
    fusion_run(&g_f, k_all_ok, T0);
    uint16_t d[TOF_NUM_SECTORS];
    for (uint32_t t = T0 + 66u * MS; t <= T0 + 500u * MS; t += 66u * MS) {
        frame_empty(TOF_SENSOR_C, t);
        zone_target(TOF_SENSOR_C, 1u, 2u, 2000, 255u); /* invalid */
        push(TOF_SENSOR_C);
        fusion_run(&g_f, k_all_ok, t);
        fusion_export(&g_f, t, d);
        TEST_ASSERT_UINT16_WITHIN(1u, expect_cm(1u, 2.0f), d[1]);
    }
    TEST_ASSERT_EQUAL_UINT32(0u, g_f.hold_expired_total);
    frame_empty(TOF_SENSOR_C, T0 + 520u * MS);
    push(TOF_SENSOR_C);
    fusion_run(&g_f, k_all_ok, T0 + 520u * MS);
    fusion_export(&g_f, T0 + 520u * MS, d);
    TEST_ASSERT_EQUAL_UINT16(FUSION_DIST_UNKNOWN, d[1]);
    TEST_ASSERT_EQUAL_UINT32(4u, g_f.hold_expired_total); /* sectors 0..3 */
}

/* A closer obstacle replaces the held one at once; a farther one after DIST_RELAX_FRAMES. */
void test_MAP13_map_closer_immediate_farther_after_relax(void)
{
    const uint32_t ts[4] = {T0, T0 + 66u * MS, T0 + 133u * MS, T0 + 200u * MS};
    const int16_t mm[4] = {2000, 1500, 2500, 2500};
    uint16_t d[TOF_NUM_SECTORS];
    uint16_t got[4];
    for (unsigned n = 0; n < 4u; n++) {
        frame_empty(TOF_SENSOR_C, ts[n]);
        zone_target(TOF_SENSOR_C, 1u, 2u, mm[n], 5u);
        push(TOF_SENSOR_C);
        fusion_run(&g_f, k_all_ok, ts[n]);
        fusion_export(&g_f, ts[n], d);
        got[n] = d[1];
    }
    TEST_ASSERT_UINT16_WITHIN(1u, expect_cm(1u, 2.0f), got[0]);
    TEST_ASSERT_UINT16_WITHIN(1u, expect_cm(1u, 1.5f), got[1]);
    TEST_ASSERT_UINT16_WITHIN(1u, expect_cm(1u, 1.5f), got[2]);
    TEST_ASSERT_UINT16_WITHIN(1u, expect_cm(1u, 2.5f), got[3]);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_MAP09_window_from_params);
    RUN_TEST(test_MAP05_in_fov_sectors_default_mounts);
    RUN_TEST(test_MAP05_out_of_fov_exports_65535);
    RUN_TEST(test_MAP05_push_rejects_wrong_resolution_or_sensor);
    RUN_TEST(test_MAP05_zone_covers_all_sectors_of_extent_without_interpolation);
    RUN_TEST(test_MAP05_cal_tol_zero_narrows_coverage);
    RUN_TEST(test_MAP05_overlap_nearest_valid_target_wins);
    RUN_TEST(test_MAP05_invalid_zone_does_not_cancel_other_sensor);
    RUN_TEST(test_MAP05_clr_requires_all_zones);
    RUN_TEST(test_MAP05_range_limits_beyond_max_clr_below_min_inv);
    RUN_TEST(test_MAP05_high_ambient_blocks_clr);
    RUN_TEST(test_MAP05_clear_disabled_empty_zones_stay_unknown);
    RUN_TEST(test_MAP04_horizontal_distance_of_elevated_zone);
    RUN_TEST(test_MAP04_mount_offset_in_distance);
    RUN_TEST(test_MAP04_row_mask_excludes_rows);
    RUN_TEST(test_MAP09_frames_within_window_fused_with_oldest_time);
    RUN_TEST(test_MAP09_ready_after_window_without_all_sensors);
    RUN_TEST(test_MAP09_frame_older_than_window_is_dropped);
    RUN_TEST(test_MAP09_stale_after_max_age);
    RUN_TEST(test_MAP10_failed_sensor_zones_are_inv);
    RUN_TEST(test_MAP10_degraded_sensor_still_reports_obstacles);
    RUN_TEST(test_MAP10_valid_fraction_of_frame);
    RUN_TEST(test_MAP11_map_state_ok_degraded_lost);
    RUN_TEST(test_MAP13_map_holds_obstacle_on_invalid_frames_then_unknown);
    RUN_TEST(test_MAP13_map_closer_immediate_farther_after_relax);
    return UNITY_END();
}
