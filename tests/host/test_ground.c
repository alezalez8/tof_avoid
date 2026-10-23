#include <math.h>
#include <string.h>

#include "core/fusion.h"
#include "core/ground.h"
#include "unity.h"

/* Zone rows of a 4x4 matrix (i = 0 top): centre elevations +16.9, +5.6, -5.6, -16.9 deg. */
#define ROW_TOP 0u
#define ROW_UP 1u
#define ROW_DOWN 2u
#define ROW_BOTTOM 3u
#define COL 1u

static geo_mount_t g_m;
static ground_ctx_t g_g;

static float deg(float d)
{
    return geo_deg_to_rad(d);
}

static void ctx(float pitch_deg, float roll_deg, float h_body)
{
    g_g.valid = true;
    g_g.pitch_rad = deg(pitch_deg);
    g_g.roll_rad = deg(roll_deg);
    g_g.h_body_m = h_body;
    g_g.tol_m = 0.15f;
    g_g.cal_tol_deg = 2.0f;
    g_g.cal_tol_el_deg = 0.5f;
}

void setUp(void)
{
    const geo_vec3_t t0 = {0.0f, 0.0f, 0.0f};
    geo_mount_init(&g_m, 0.0f, 0.0f, 0.0f, t0, 4u, GEO_ORIENT_IDENTITY);
    ctx(0.0f, 0.0f, 0.20f);
}

void tearDown(void) {}

/* Range along the centre ray of row i where it meets the ground, for optics at height h. */
static float ground_range(uint8_t n, uint8_t i, float pitch_deg, float h)
{
    return h / sinf(deg(-(geo_zone_beta_deg(n, i) + pitch_deg)));
}

/* Brute-force minimum of h_p over the zone box, for checking the exact maximum. */
static float brute_min_height(const geo_mount_t *m, uint8_t i, uint8_t j, float r, const ground_ctx_t *g)
{
    const float half_a = 0.5f * geo_zone_width_deg(m->n) + g->cal_tol_deg;
    const float half_b = 0.5f * geo_zone_width_deg(m->n) + g->cal_tol_el_deg;
    const float a_c = geo_zone_alpha_deg(m->n, j);
    const float b_c = geo_zone_beta_deg(m->n, i);
    float best = 1e9f;
    for (int ia = 0; ia <= 60; ia++) {
        for (int ib = 0; ib <= 60; ib++) {
            const float a = a_c - half_a + (2.0f * half_a) * (float)ia / 60.0f;
            const float b = b_c - half_b + (2.0f * half_b) * (float)ib / 60.0f;
            const geo_vec3_t d = geo_dir_sensor(a, b);
            const geo_vec3_t p = {d.x * r, d.y * r, d.z * r};
            const float h = geo_point_height(m, p, g->pitch_rad, g->roll_rad, g->h_body_m);
            best = (h < best) ? h : best;
        }
    }
    return best;
}

/* MAP-14: the exact minimum over the zone extent equals a dense sampling of it (mounts, attitudes,
 * offsets, both resolutions). */
void test_MAP14_min_height_matches_sampling(void)
{
    const float yaws[] = {-40.0f, 0.0f, 30.0f};
    const float att[][2] = {{0.0f, 0.0f}, {-15.0f, 0.0f}, {15.0f, 0.0f}, {5.0f, -15.0f}, {-10.0f, 12.0f}};
    const uint8_t ns[] = {4u, 8u};
    for (unsigned y = 0; y < sizeof yaws / sizeof yaws[0]; y++) {
        for (unsigned a = 0; a < sizeof att / sizeof att[0]; a++) {
            for (unsigned k = 0; k < 2u; k++) {
                const geo_vec3_t t = {0.05f, -0.03f, 0.04f};
                geo_mount_t m;
                geo_mount_init(&m, yaws[y], 1.0f, -1.0f, t, ns[k], GEO_ORIENT_IDENTITY);
                ctx(att[a][0], att[a][1], 1.0f);
                for (uint8_t i = 0; i < ns[k]; i += 3u) {
                    for (uint8_t j = 0; j < ns[k]; j += 3u) {
                        const float exact = ground_min_height(&m, i, j, 2.0f, &g_g);
                        const float brute = brute_min_height(&m, i, j, 2.0f, &g_g);
                        TEST_ASSERT_TRUE(exact <= brute + 1e-4f);
                        TEST_ASSERT_FLOAT_WITHIN(0.01f, brute, exact);
                    }
                }
            }
        }
    }
}

/* MAP-14: optics at 0.20 m on the ground, pitch 0. The two bottom rows see the ground (centre of the
 * bottom row at ~0.66 m, second row at ~2.0 m horizontally): ground-compatible. */
void test_MAP14_ground_20cm_bottom_rows_are_ground(void)
{
    const float r3 = ground_range(4u, ROW_BOTTOM, 0.0f, 0.20f);
    const float r2 = ground_range(4u, ROW_DOWN, 0.0f, 0.20f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.66f, r3 * cosf(deg(16.875f)));
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.03f, r2 * cosf(deg(5.625f)));
    TEST_ASSERT_TRUE(ground_target_compatible(&g_m, ROW_BOTTOM, COL, r3, &g_g));
    TEST_ASSERT_TRUE(ground_target_compatible(&g_m, ROW_DOWN, COL, r2, &g_g));
}

/* MAP-14: a target in the top row (all rays above the horizon) is never ground - a real obstacle. */
void test_MAP14_ground_20cm_top_row_is_obstacle(void)
{
    for (float r = 0.1f; r < 4.0f; r += 0.1f) {
        TEST_ASSERT_FALSE(ground_target_compatible(&g_m, ROW_TOP, COL, r, &g_g));
    }
}

/* MAP-14, MAP-14b: the check uses the zone extent, not the centre ray. Row 1 points 5.6 deg up (never
 * ground by its centre ray); its extent with CAL_TOL_EL_DEG = 0.5 reaches -0.5 deg, so from ~5.7 m on
 * some direction is within GROUND_TOL_M of the ground (upper edge at the horizon: a MAP-19 limitation).
 * At 2 m it is a real obstacle. */
void test_MAP14_extent_not_centre_ray(void)
{
    const geo_vec3_t c = geo_dir_sensor(geo_zone_alpha_deg(4u, COL), geo_zone_beta_deg(4u, ROW_UP));
    const float centre_h = geo_point_height(&g_m, (geo_vec3_t){c.x * 6.0f, c.y * 6.0f, c.z * 6.0f}, 0.0f,
                                            0.0f, 0.20f);
    TEST_ASSERT_TRUE(centre_h > 0.15f); /* centre ray: no ground */
    TEST_ASSERT_TRUE(ground_target_compatible(&g_m, ROW_UP, COL, 6.0f, &g_g));
    TEST_ASSERT_FALSE(ground_target_compatible(&g_m, ROW_UP, COL, 2.0f, &g_g));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.20f - 2.0f * sinf(deg(0.5f)), ground_min_height(&g_m, ROW_UP, COL, 2.0f, &g_g));
    /* Without the elevation tolerance the extent stops at 0 deg: never below 0.20 m. */
    g_g.cal_tol_el_deg = 0.0f;
    TEST_ASSERT_FALSE(ground_target_compatible(&g_m, ROW_UP, COL, 6.0f, &g_g));
}

/* MAP-14b: CAL_TOL_DEG widens the azimuth only; the elevation uses CAL_TOL_EL_DEG. */
void test_MAP14b_tolerances_by_axis(void)
{
    g_g.cal_tol_deg = 10.0f; /* azimuth: no effect on the lowest direction of a level sensor */
    TEST_ASSERT_FALSE(ground_target_compatible(&g_m, ROW_UP, COL, 2.0f, &g_g));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.20f - 2.0f * sinf(deg(0.5f)), ground_min_height(&g_m, ROW_UP, COL, 2.0f, &g_g));
    g_g.cal_tol_el_deg = 2.0f; /* the E1b behaviour, for comparison: ground from ~1.43 m */
    TEST_ASSERT_TRUE(ground_target_compatible(&g_m, ROW_UP, COL, 2.0f, &g_g));
}

/* MAP-14: GROUND_TOL_M is the threshold (<=). */
void test_MAP14_tolerance_threshold(void)
{
    const float h = ground_min_height(&g_m, ROW_UP, COL, 2.0f, &g_g);
    g_g.tol_m = h + 1e-4f;
    TEST_ASSERT_TRUE(ground_target_compatible(&g_m, ROW_UP, COL, 2.0f, &g_g));
    g_g.tol_m = h - 1e-4f;
    TEST_ASSERT_FALSE(ground_target_compatible(&g_m, ROW_UP, COL, 2.0f, &g_g));
}

/* MAP-14, MAP-02: pitch -15 deg (nose down) at 1 m brings the ground closer: the bottom row hits it at
 * ~1.6 m horizontally and is ground; the top row at 2 m is still an obstacle. */
void test_MAP14_pitch_nose_down_15(void)
{
    ctx(-15.0f, 0.0f, 1.0f);
    const float r = ground_range(4u, ROW_BOTTOM, -15.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.89f, r);
    TEST_ASSERT_TRUE(ground_target_compatible(&g_m, ROW_BOTTOM, COL, r, &g_g));
    TEST_ASSERT_FALSE(ground_target_compatible(&g_m, ROW_TOP, COL, 2.0f, &g_g));
    /* At pitch 0 the same bottom-row target at 1.89 m is an obstacle (ground starts at ~2.05 m). */
    ctx(0.0f, 0.0f, 1.0f);
    TEST_ASSERT_FALSE(ground_target_compatible(&g_m, ROW_BOTTOM, COL, r, &g_g));
}

/* MAP-14: pitch +15 deg (nose up) at 1 m lifts the bottom row: a target at 3 m is an obstacle, while
 * at pitch 0 the same target is ground. */
void test_MAP14_pitch_nose_up_15(void)
{
    ctx(15.0f, 0.0f, 1.0f);
    TEST_ASSERT_FALSE(ground_target_compatible(&g_m, ROW_BOTTOM, COL, 3.0f, &g_g));
    /* Off-axis rays of the zone are lifted less than the centre column: about 0.49 m, not
     * 1 - 3 sin(9.5 deg) = 0.50 m. */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, brute_min_height(&g_m, ROW_BOTTOM, COL, 3.0f, &g_g),
                             ground_min_height(&g_m, ROW_BOTTOM, COL, 3.0f, &g_g));
    ctx(0.0f, 0.0f, 1.0f);
    TEST_ASSERT_TRUE(ground_target_compatible(&g_m, ROW_BOTTOM, COL, 3.0f, &g_g));
}

/* MAP-14: roll right side down tilts a side sensor towards the ground. */
void test_MAP14_roll_side_sensor(void)
{
    const geo_vec3_t t0 = {0.0f, 0.0f, 0.0f};
    geo_mount_t right;
    geo_mount_init(&right, 30.0f, 0.0f, 0.0f, t0, 4u, GEO_ORIENT_IDENTITY);
    ctx(0.0f, 0.0f, 1.0f);
    const float level = ground_min_height(&right, ROW_DOWN, 3u, 3.0f, &g_g);
    ctx(0.0f, 15.0f, 1.0f);
    TEST_ASSERT_TRUE(ground_min_height(&right, ROW_DOWN, 3u, 3.0f, &g_g) < level);
    ctx(0.0f, -15.0f, 1.0f);
    TEST_ASSERT_TRUE(ground_min_height(&right, ROW_DOWN, 3u, 3.0f, &g_g) > level);
}

/* MAP-14: without valid orientation (or without a context) the flag is never set. */
void test_MAP14_no_orientation_no_flag(void)
{
    const float r3 = ground_range(4u, ROW_BOTTOM, 0.0f, 0.20f);
    g_g.valid = false;
    TEST_ASSERT_FALSE(ground_target_compatible(&g_m, ROW_BOTTOM, COL, r3, &g_g));
    TEST_ASSERT_FALSE(ground_target_compatible(&g_m, ROW_BOTTOM, COL, r3, NULL));
}

/* MAP-14a: heights refer to the body centre; with TOF_HEIGHT_GROUND_M corrected by MOUNT_XYZ_M the
 * result does not depend on the sensor offset. */
void test_MAP14a_offset_cancels_on_ground(void)
{
    const geo_vec3_t t = {0.10f, 0.05f, 0.04f};
    geo_mount_t m;
    geo_mount_init(&m, 0.0f, 0.0f, 0.0f, t, 4u, GEO_ORIENT_IDENTITY);
    const ground_height_in_t hin = {false, 0.0f, {0.0f, 0.0f, 0.0f}, 0.20f};
    float h = 0.0f;
    TEST_ASSERT_EQUAL_INT(GROUND_H_PARAM, ground_body_height(PHASE_GROUND, &hin, &m, 0.0f, 0.0f, &h));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.24f, h);
    const float without_offset = ground_min_height(&g_m, ROW_BOTTOM, COL, 0.5f, &g_g); /* h_body 0.20 */
    ctx(0.0f, 0.0f, h);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, without_offset, ground_min_height(&m, ROW_BOTTOM, COL, 0.5f, &g_g));
}

/* MAP-15, MAP-15b: height source by phase; the ground parameter is never used in the air. */
void test_MAP15_height_source_by_phase(void)
{
    TEST_ASSERT_EQUAL_INT(GROUND_H_RANGEFINDER, ground_height_source(PHASE_FLIGHT, true));
    TEST_ASSERT_EQUAL_INT(GROUND_H_NONE, ground_height_source(PHASE_FLIGHT, false));
    TEST_ASSERT_EQUAL_INT(GROUND_H_PARAM, ground_height_source(PHASE_GROUND, true));
    TEST_ASSERT_EQUAL_INT(GROUND_H_PARAM, ground_height_source(PHASE_ARMED_GROUND, true));
    TEST_ASSERT_EQUAL_INT(GROUND_H_PARAM, ground_height_source(PHASE_GROUND_RECOVER, false));
    TEST_ASSERT_EQUAL_INT(GROUND_H_RANGEFINDER, ground_height_source(PHASE_TAKEOFF, true));
    TEST_ASSERT_EQUAL_INT(GROUND_H_NONE, ground_height_source(PHASE_TAKEOFF, false));
    TEST_ASSERT_EQUAL_INT(GROUND_H_RANGEFINDER, ground_height_source(PHASE_LANDING, true));
    TEST_ASSERT_EQUAL_INT(GROUND_H_NONE, ground_height_source(PHASE_LANDING, false));
    TEST_ASSERT_EQUAL_INT(GROUND_H_RANGEFINDER, ground_height_source(PHASE_UNKNOWN, true));
    TEST_ASSERT_EQUAL_INT(GROUND_H_NONE, ground_height_source(PHASE_UNKNOWN, false));
}

/* MAP-15, MAP-14a: the rangefinder height is corrected by RF_XYZ_M and attitude; no source leaves the
 * output untouched. */
void test_MAP15_body_height_values(void)
{
    const ground_height_in_t hin = {true, 2.0f, {0.0f, 0.0f, 0.05f}, 0.20f};
    float h = -1.0f;
    TEST_ASSERT_EQUAL_INT(GROUND_H_RANGEFINDER, ground_body_height(PHASE_FLIGHT, &hin, &g_m, 0.0f, 0.0f, &h));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 2.05f, h);
    TEST_ASSERT_EQUAL_INT(GROUND_H_RANGEFINDER, ground_body_height(PHASE_FLIGHT, &hin, &g_m, deg(10.0f), 0.0f, &h));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f * cosf(deg(10.0f)) + 0.05f * cosf(deg(10.0f)), h);
    const ground_height_in_t none = {false, 0.0f, {0.0f, 0.0f, 0.0f}, 0.20f};
    h = -1.0f;
    TEST_ASSERT_EQUAL_INT(GROUND_H_NONE, ground_body_height(PHASE_FLIGHT, &none, &g_m, 0.0f, 0.0f, &h));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, h);
}

/* ---- MAP-18 and the two map views in fusion ---- */

static params_t g_p;
static fusion_t g_f;
static tof_frame_t g_fr;

static void fusion_setup(void)
{
    params_set_defaults(&g_p);
    fusion_init(&g_f, &g_p);
    memset(&g_fr, 0, sizeof g_fr);
    g_fr.sensor_id = TOF_SENSOR_C;
    g_fr.res = 4u;
    g_fr.freq_hz = 15u;
}

static void set_zone(uint8_t i, uint8_t j, uint8_t n, float r0_m, float r1_m)
{
    tof_zone_t *z = &g_fr.zone[i * 4u + j];
    z->nb_target_detected = n;
    z->target[0].distance_mm = (int16_t)(r0_m * 1000.0f);
    z->target[0].target_status = 5u;
    z->target[1].distance_mm = (int16_t)(r1_m * 1000.0f);
    z->target[1].target_status = 5u;
}

static void run_steps(uint32_t n)
{
    const sensor_health_t h[TOF_NUM_SENSORS] = {SENSOR_OK, SENSOR_OK, SENSOR_OK};
    for (uint32_t k = 0; k < n; k++) {
        g_fr.t_ready_us = k * 66000u;
        TEST_ASSERT_TRUE(fusion_push_frame(&g_f, &g_fr, &g_g));
        fusion_run(&g_f, h, g_fr.t_ready_us);
    }
}

static uint16_t min_export(fusion_view_t v)
{
    uint16_t d[TOF_NUM_SECTORS];
    fusion_export_view(&g_f, v, g_fr.t_ready_us, d);
    uint16_t m = 65535u;
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        m = (d[k] < m) ? d[k] : m;
    }
    return m;
}

static uint16_t horiz_cm(uint8_t i, uint8_t j, float r)
{
    float d = 0.0f;
    float az = 0.0f;
    geo_zone_target(&g_f.mount[TOF_SENSOR_C], i, j, r, &d, &az);
    return (uint16_t)(d * 100.0f);
}

/* MAP-18, MAP-17: ground at 20 cm in the bottom row is GROUND_SUSPECT: kept in the ALL view, 65535 in
 * the REAL view. */
void test_MAP18_ground_zone_flagged(void)
{
    fusion_setup();
    const float r3 = ground_range(4u, ROW_BOTTOM, 0.0f, 0.20f);
    set_zone(ROW_BOTTOM, COL, 1u, r3, 0.0f);
    run_steps(1u);
    const fusion_zone_t *z = &g_f.frame[TOF_SENSOR_C].zone[ROW_BOTTOM * 4u + COL];
    TEST_ASSERT_TRUE(z->ground);
    TEST_ASSERT_EQUAL_UINT8(ZONE_OCC, z->cls);
    TEST_ASSERT_EQUAL_UINT8(ZONE_INV, z->cls_real);
    TEST_ASSERT_EQUAL_UINT8(1u, g_f.frame[TOF_SENSOR_C].n_ground_zones);
    TEST_ASSERT_UINT16_WITHIN(1u, horiz_cm(ROW_BOTTOM, COL, r3), min_export(FUSION_VIEW_ALL));
    TEST_ASSERT_EQUAL_UINT16(65535u, min_export(FUSION_VIEW_REAL));
    /* The held cell of the ALL view carries the GROUND_SUSPECT flag. */
    bool flagged = false;
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        flagged = flagged || (g_f.cell[k].state == CELL_OCCUPIED && g_f.cell[k].ground_suspect);
    }
    TEST_ASSERT_TRUE(flagged);
}

/* MAP-18: mixed zone "ground + low target" in flight (1 m, pitch 0, bottom row): the nearer target at
 * 1.5 m is incompatible with the ground, the farther one is the ground. The zone is OCCUPIED by the
 * real target in both views and is not GROUND_SUSPECT. */
void test_MAP18_mixed_zone_ground_and_low_target(void)
{
    fusion_setup();
    ctx(0.0f, 0.0f, 1.0f);
    const float r_ground = ground_range(4u, ROW_BOTTOM, 0.0f, 1.0f);
    TEST_ASSERT_FALSE(ground_target_compatible(&g_f.mount[TOF_SENSOR_C], ROW_BOTTOM, COL, 1.5f, &g_g));
    TEST_ASSERT_TRUE(ground_target_compatible(&g_f.mount[TOF_SENSOR_C], ROW_BOTTOM, COL, r_ground, &g_g));
    set_zone(ROW_BOTTOM, COL, 2u, 1.5f, r_ground);
    run_steps(1u);
    const fusion_zone_t *z = &g_f.frame[TOF_SENSOR_C].zone[ROW_BOTTOM * 4u + COL];
    TEST_ASSERT_FALSE(z->ground);
    TEST_ASSERT_EQUAL_UINT8(ZONE_OCC, z->cls_real);
    TEST_ASSERT_UINT16_WITHIN(1u, horiz_cm(ROW_BOTTOM, COL, 1.5f), min_export(FUSION_VIEW_REAL));
    TEST_ASSERT_UINT16_WITHIN(1u, horiz_cm(ROW_BOTTOM, COL, 1.5f), min_export(FUSION_VIEW_ALL));
}

/* MAP-18, MAP-19: on the ground (0.20 m) a low object at 0.4 m in the bottom row is
 * indistinguishable from the ground by this model; with the ground behind it both targets are
 * ground-compatible and the zone is GROUND_SUSPECT. Documented limitation until T-25 / T-26. */
void test_MAP19_low_object_near_ground_not_distinguished(void)
{
    fusion_setup();
    const float r3 = ground_range(4u, ROW_BOTTOM, 0.0f, 0.20f);
    set_zone(ROW_BOTTOM, COL, 2u, 0.40f, r3);
    run_steps(1u);
    TEST_ASSERT_TRUE(g_f.frame[TOF_SENSOR_C].zone[ROW_BOTTOM * 4u + COL].ground);
    TEST_ASSERT_EQUAL_UINT16(65535u, min_export(FUSION_VIEW_REAL));
}

/* MAP-17: a real obstacle behind the ground in the same sector (wall at 2 m in the top row, ground in
 * the bottom row of the same column) is exported in the REAL view; the ALL view exports the nearer
 * ground point. */
void test_MAP17_real_obstacle_behind_ground_exported(void)
{
    fusion_setup();
    const float r3 = ground_range(4u, ROW_BOTTOM, 0.0f, 0.20f);
    set_zone(ROW_BOTTOM, COL, 1u, r3, 0.0f);
    set_zone(ROW_TOP, COL, 1u, 2.0f, 0.0f);
    run_steps(1u);
    TEST_ASSERT_UINT16_WITHIN(1u, horiz_cm(ROW_TOP, COL, 2.0f), min_export(FUSION_VIEW_REAL));
    TEST_ASSERT_UINT16_WITHIN(1u, horiz_cm(ROW_BOTTOM, COL, r3), min_export(FUSION_VIEW_ALL));
}

/* MAP-14: no valid orientation - nothing is GROUND_SUSPECT, both views are equal. */
void test_MAP14_no_orientation_views_equal(void)
{
    fusion_setup();
    g_g.valid = false;
    const float r3 = ground_range(4u, ROW_BOTTOM, 0.0f, 0.20f);
    set_zone(ROW_BOTTOM, COL, 1u, r3, 0.0f);
    run_steps(1u);
    TEST_ASSERT_FALSE(g_f.frame[TOF_SENSOR_C].zone[ROW_BOTTOM * 4u + COL].ground);
    TEST_ASSERT_EQUAL_UINT16(min_export(FUSION_VIEW_ALL), min_export(FUSION_VIEW_REAL));
    TEST_ASSERT_TRUE(min_export(FUSION_VIEW_REAL) < 65535u);
}

/* MAP-13 in the REAL view: a held real obstacle replaced by ground data is held (ground is INV there),
 * then expires after HOLD_MAX_MS with a log flag. */
void test_MAP13_real_view_ground_does_not_release(void)
{
    fusion_setup();
    const sensor_health_t h[TOF_NUM_SENSORS] = {SENSOR_OK, SENSOR_OK, SENSOR_OK};
    TEST_ASSERT_FALSE(ground_target_compatible(&g_f.mount[TOF_SENSOR_C], ROW_DOWN, COL, 0.2f, &g_g));
    TEST_ASSERT_TRUE(ground_target_compatible(&g_f.mount[TOF_SENSOR_C], ROW_DOWN, COL, 1.0f, &g_g));
    set_zone(ROW_DOWN, COL, 1u, 0.2f, 0.0f); /* real, row 2 at 0.2 m */
    run_steps(1u);
    const uint16_t held = min_export(FUSION_VIEW_REAL);
    TEST_ASSERT_TRUE(held < 65535u);
    set_zone(ROW_DOWN, COL, 1u, 1.0f, 0.0f); /* now the ground at 1 m */
    g_fr.t_ready_us = 100000u;
    (void)fusion_push_frame(&g_f, &g_fr, &g_g);
    fusion_run(&g_f, h, 100000u);
    TEST_ASSERT_EQUAL_UINT16(held, min_export(FUSION_VIEW_REAL));
    g_fr.t_ready_us = 700000u;
    (void)fusion_push_frame(&g_f, &g_fr, &g_g);
    fusion_run(&g_f, h, 700000u);
    TEST_ASSERT_EQUAL_UINT16(65535u, min_export(FUSION_VIEW_REAL));
    bool expired = false;
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        expired = expired || g_f.hold_expired[FUSION_VIEW_REAL][k];
    }
    TEST_ASSERT_TRUE(expired);
}

/* MAP-16 (GROUND_RECOVER): reset drops every hold in both views. */
void test_MAP16_reset_cells(void)
{
    fusion_setup();
    set_zone(ROW_TOP, COL, 1u, 2.0f, 0.0f);
    run_steps(1u);
    TEST_ASSERT_TRUE(min_export(FUSION_VIEW_ALL) < 65535u);
    fusion_reset_cells(&g_f);
    TEST_ASSERT_EQUAL_UINT16(65535u, min_export(FUSION_VIEW_ALL));
    TEST_ASSERT_EQUAL_UINT16(65535u, min_export(FUSION_VIEW_REAL));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_MAP14_min_height_matches_sampling);
    RUN_TEST(test_MAP14_ground_20cm_bottom_rows_are_ground);
    RUN_TEST(test_MAP14_ground_20cm_top_row_is_obstacle);
    RUN_TEST(test_MAP14_extent_not_centre_ray);
    RUN_TEST(test_MAP14b_tolerances_by_axis);
    RUN_TEST(test_MAP14_tolerance_threshold);
    RUN_TEST(test_MAP14_pitch_nose_down_15);
    RUN_TEST(test_MAP14_pitch_nose_up_15);
    RUN_TEST(test_MAP14_roll_side_sensor);
    RUN_TEST(test_MAP14_no_orientation_no_flag);
    RUN_TEST(test_MAP14a_offset_cancels_on_ground);
    RUN_TEST(test_MAP15_height_source_by_phase);
    RUN_TEST(test_MAP15_body_height_values);
    RUN_TEST(test_MAP18_ground_zone_flagged);
    RUN_TEST(test_MAP18_mixed_zone_ground_and_low_target);
    RUN_TEST(test_MAP19_low_object_near_ground_not_distinguished);
    RUN_TEST(test_MAP17_real_obstacle_behind_ground_exported);
    RUN_TEST(test_MAP14_no_orientation_views_equal);
    RUN_TEST(test_MAP13_real_view_ground_does_not_release);
    RUN_TEST(test_MAP16_reset_cells);
    return UNITY_END();
}
