#include <math.h>

#include "core/geometry.h"
#include "unity.h"

#define TOL 1e-4f
#define TOL_DEG 1e-3f

void setUp(void) {}
void tearDown(void) {}

static float sind(float deg)
{
    return sinf(geo_deg_to_rad(deg));
}

static float cosd(float deg)
{
    return cosf(geo_deg_to_rad(deg));
}

static float tand(float deg)
{
    return tanf(geo_deg_to_rad(deg));
}

static void assert_vec(geo_vec3_t e, geo_vec3_t a, float tol)
{
    TEST_ASSERT_FLOAT_WITHIN(tol, e.x, a.x);
    TEST_ASSERT_FLOAT_WITHIN(tol, e.y, a.y);
    TEST_ASSERT_FLOAT_WITHIN(tol, e.z, a.z);
}

static geo_mount_t mount(float yaw, float pitch, float roll, uint8_t n)
{
    geo_mount_t m;
    const geo_vec3_t t = {0.0f, 0.0f, 0.0f};
    geo_mount_init(&m, yaw, pitch, roll, t, n, GEO_ORIENT_IDENTITY);
    return m;
}

/* ---- Angle helpers ---- */

void test_MAP02_deg_to_rad(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, geo_deg_to_rad(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, GEO_PI_F, geo_deg_to_rad(180.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -GEO_PI_F / 2.0f, geo_deg_to_rad(-90.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 90.0f, geo_rad_to_deg(GEO_PI_F / 2.0f));
}

void test_MAP02_wrap_keeps_range_0_360(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, geo_wrap_deg_360(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 45.0f, geo_wrap_deg_360(45.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, geo_wrap_deg_360(360.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.0f, geo_wrap_deg_360(725.0f));
}

void test_MAP02_wrap_negative_angles(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 355.0f, geo_wrap_deg_360(-5.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 330.0f, geo_wrap_deg_360(-30.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, geo_wrap_deg_360(-360.0f));
}

void test_MAP02_wrap_tiny_negative_is_below_360(void)
{
    const float r = geo_wrap_deg_360(-1e-6f);
    TEST_ASSERT_TRUE(r >= 0.0f);
    TEST_ASSERT_TRUE(r < 360.0f);
}

void test_MAP02_wrap_180(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, geo_wrap_deg_180(360.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -5.0f, geo_wrap_deg_180(355.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 170.0f, geo_wrap_deg_180(-190.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -180.0f, geo_wrap_deg_180(180.0f));
}

/* ---- MAP-01: frames and signs ---- */

/* alpha_j = (j - (N-1)/2) * 45/N, beta_i = ((N-1)/2 - i) * 45/N; i = 0 is the top row. */
void test_MAP01_zone_angles_4x4_and_8x8(void)
{
    TEST_ASSERT_FLOAT_WITHIN(TOL, -16.875f, geo_zone_alpha_deg(4u, 0u));
    TEST_ASSERT_FLOAT_WITHIN(TOL, -5.625f, geo_zone_alpha_deg(4u, 1u));
    TEST_ASSERT_FLOAT_WITHIN(TOL, 5.625f, geo_zone_alpha_deg(4u, 2u));
    TEST_ASSERT_FLOAT_WITHIN(TOL, 16.875f, geo_zone_alpha_deg(4u, 3u));
    TEST_ASSERT_FLOAT_WITHIN(TOL, 16.875f, geo_zone_beta_deg(4u, 0u));
    TEST_ASSERT_FLOAT_WITHIN(TOL, -16.875f, geo_zone_beta_deg(4u, 3u));
    TEST_ASSERT_FLOAT_WITHIN(TOL, -19.6875f, geo_zone_alpha_deg(8u, 0u));
    TEST_ASSERT_FLOAT_WITHIN(TOL, 19.6875f, geo_zone_alpha_deg(8u, 7u));
    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.8125f, geo_zone_beta_deg(8u, 3u));
    TEST_ASSERT_FLOAT_WITHIN(TOL, 45.0f / 4.0f, geo_zone_width_deg(4u));
    TEST_ASSERT_FLOAT_WITHIN(TOL, 45.0f / 8.0f, geo_zone_width_deg(8u));
}

/* p_S = r [cos b cos a, cos b sin a, -sin b]: x along the axis, y right, z down. */
void test_MAP01_sensor_frame_signs(void)
{
    const geo_vec3_t fwd = {1.0f, 0.0f, 0.0f};
    assert_vec(fwd, geo_dir_sensor(0.0f, 0.0f), TOL);
    const geo_vec3_t right = geo_dir_sensor(10.0f, 0.0f);
    TEST_ASSERT_TRUE(right.y > 0.0f); /* positive alpha is to the right */
    const geo_vec3_t up = geo_dir_sensor(0.0f, 10.0f);
    TEST_ASSERT_TRUE(up.z < 0.0f); /* positive beta is up, z is down */
    const geo_vec3_t e = {cosd(16.875f) * cosd(-5.625f), cosd(16.875f) * sind(-5.625f),
                          -sind(16.875f)};
    assert_vec(e, geo_dir_sensor(-5.625f, 16.875f), TOL);
}

/* R_y(theta) maps [1, 0, 0] to [cos theta, 0, -sin theta]; R_x, R_z right-handed. */
void test_MAP01_elementary_rotations(void)
{
    const geo_vec3_t ex = {1.0f, 0.0f, 0.0f};
    const geo_vec3_t ey = {0.0f, 1.0f, 0.0f};
    const float a = geo_deg_to_rad(20.0f);
    const geo_vec3_t ry = {cosd(20.0f), 0.0f, -sind(20.0f)};
    assert_vec(ry, geo_mat_apply(geo_rot_y(a), ex), TOL);
    const geo_vec3_t rz = {cosd(20.0f), sind(20.0f), 0.0f};
    assert_vec(rz, geo_mat_apply(geo_rot_z(a), ex), TOL);
    const geo_vec3_t rx = {0.0f, cosd(20.0f), sind(20.0f)};
    assert_vec(rx, geo_mat_apply(geo_rot_x(a), ey), TOL);
}

/* ---- MAP-02: mounting yaw / pitch / roll, both signs ---- */

void test_MAP02_mount_yaw_positive_is_right(void)
{
    const geo_mount_t m = mount(30.0f, 0.0f, 0.0f, 4u);
    const geo_vec3_t p = geo_sensor_to_body(&m, geo_dir_sensor(0.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 30.0f, geo_azimuth_deg(p));
    TEST_ASSERT_TRUE(p.y > 0.0f);
}

void test_MAP02_mount_yaw_negative_is_left(void)
{
    const geo_mount_t m = mount(-30.0f, 0.0f, 0.0f, 4u);
    const geo_vec3_t p = geo_sensor_to_body(&m, geo_dir_sensor(0.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, -30.0f, geo_azimuth_deg(p));
    TEST_ASSERT_TRUE(p.y < 0.0f);
}

void test_MAP02_mount_pitch_positive_is_nose_up(void)
{
    const geo_mount_t up = mount(0.0f, 5.0f, 0.0f, 4u);
    const geo_vec3_t e_up = {cosd(5.0f), 0.0f, -sind(5.0f)};
    assert_vec(e_up, geo_sensor_to_body(&up, geo_dir_sensor(0.0f, 0.0f)), TOL);
    const geo_mount_t dn = mount(0.0f, -5.0f, 0.0f, 4u);
    const geo_vec3_t e_dn = {cosd(5.0f), 0.0f, sind(5.0f)};
    assert_vec(e_dn, geo_sensor_to_body(&dn, geo_dir_sensor(0.0f, 0.0f)), TOL);
}

void test_MAP02_mount_roll_positive_is_right_side_down(void)
{
    const geo_vec3_t ey = {0.0f, 1.0f, 0.0f};
    const geo_mount_t rp = mount(0.0f, 0.0f, 5.0f, 4u);
    const geo_vec3_t e_p = {0.0f, cosd(5.0f), sind(5.0f)};
    assert_vec(e_p, geo_sensor_to_body(&rp, ey), TOL);
    const geo_mount_t rn = mount(0.0f, 0.0f, -5.0f, 4u);
    const geo_vec3_t e_n = {0.0f, cosd(5.0f), -sind(5.0f)};
    assert_vec(e_n, geo_sensor_to_body(&rn, ey), TOL);
}

/* ZYX order: p_B = Rz(yaw) Ry(pitch) Rx(roll) p_S, checked against the explicit product. */
void test_MAP02_zyx_order(void)
{
    const float y = 30.0f;
    const float p = 10.0f;
    const float r = -7.0f;
    const geo_mount_t m = mount(y, p, r, 4u);
    const geo_mat3_t expect = geo_mat_mul(
        geo_rot_z(geo_deg_to_rad(y)),
        geo_mat_mul(geo_rot_y(geo_deg_to_rad(p)), geo_rot_x(geo_deg_to_rad(r))));
    const geo_vec3_t v = geo_dir_sensor(12.0f, -9.0f);
    assert_vec(geo_mat_apply(expect, v), geo_sensor_to_body(&m, v), TOL);
    /* XYZ order gives a different vector: the order matters. */
    const geo_mat3_t xyz = geo_mat_mul(
        geo_rot_x(geo_deg_to_rad(r)),
        geo_mat_mul(geo_rot_y(geo_deg_to_rad(p)), geo_rot_z(geo_deg_to_rad(y))));
    const geo_vec3_t w = geo_mat_apply(xyz, v);
    const geo_vec3_t b = geo_sensor_to_body(&m, v);
    TEST_ASSERT_TRUE(fabsf(w.x - b.x) + fabsf(w.y - b.y) + fabsf(w.z - b.z) > 1e-2f);
}

/* ---- MAP-02: sensors at -theta / 0 / +theta ---- */

void test_MAP02_three_sensors_zone_azimuths(void)
{
    const float yaw[3] = {-30.0f, 0.0f, 30.0f};
    const float alpha[4] = {-16.875f, -5.625f, 5.625f, 16.875f};
    for (unsigned s = 0; s < 3u; s++) {
        const geo_mount_t m = mount(yaw[s], 0.0f, 0.0f, 4u);
        for (uint8_t i = 0; i < 4u; i++) {
            for (uint8_t j = 0; j < 4u; j++) {
                float d = 0.0f;
                float az = 0.0f;
                geo_zone_target(&m, i, j, 2.0f, &d, &az);
                TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, yaw[s] + alpha[j], az);
                /* MAP-04: horizontal distance, not slant range */
                TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f * cosd(geo_zone_beta_deg(4u, i)), d);
            }
        }
    }
}

void test_MAP02_three_sensors_extents(void)
{
    const float yaw[3] = {-30.0f, 0.0f, 30.0f};
    for (unsigned s = 0; s < 3u; s++) {
        const geo_mount_t m = mount(yaw[s], 0.0f, 0.0f, 4u);
        geo_zone_extent_t e0;
        geo_zone_extent_t e3;
        geo_zone_extent(&m, 1u, 0u, &e0);
        geo_zone_extent(&m, 1u, 3u, &e3);
        TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, yaw[s] - 22.5f, e0.az_min_deg);
        TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, yaw[s] - 11.25f, e0.az_max_deg);
        TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, yaw[s] + 11.25f, e3.az_min_deg);
        TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, yaw[s] + 22.5f, e3.az_max_deg);
        /* Sensor-frame extent is stored too. */
        TEST_ASSERT_FLOAT_WITHIN(TOL, -22.5f, e0.alpha_min_deg);
        TEST_ASSERT_FLOAT_WITHIN(TOL, -11.25f, e0.alpha_max_deg);
        TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, e0.beta_min_deg);
        TEST_ASSERT_FLOAT_WITHIN(TOL, 11.25f, e0.beta_max_deg);
    }
}

/* Extent of a pitched / rolled sensor contains every direction inside the zone. */
void test_MAP02_extent_contains_zone_directions_when_tilted(void)
{
    const geo_mount_t m = mount(30.0f, 10.0f, -12.0f, 4u);
    const float w = geo_zone_width_deg(4u);
    for (uint8_t i = 0; i < 4u; i++) {
        for (uint8_t j = 0; j < 4u; j++) {
            geo_zone_extent_t e;
            geo_zone_extent(&m, i, j, &e);
            TEST_ASSERT_TRUE(e.az_min_deg < e.az_center_deg);
            TEST_ASSERT_TRUE(e.az_max_deg > e.az_center_deg);
            const float a0 = geo_zone_alpha_deg(4u, j) - w / 2.0f;
            const float b0 = geo_zone_beta_deg(4u, i) - w / 2.0f;
            for (int ka = 0; ka <= 10; ka++) {
                for (int kb = 0; kb <= 10; kb++) {
                    const geo_vec3_t d = geo_sensor_to_body(
                        &m, geo_dir_sensor(a0 + w * (float)ka / 10.0f, b0 + w * (float)kb / 10.0f));
                    const float az = geo_azimuth_deg(d);
                    TEST_ASSERT_TRUE(az >= e.az_min_deg - 0.05f);
                    TEST_ASSERT_TRUE(az <= e.az_max_deg + 0.05f);
                }
            }
        }
    }
}

/* Extent across the +-180 deg seam stays contiguous around the centre. */
void test_MAP02_extent_across_rear_seam(void)
{
    const geo_mount_t m = mount(180.0f, 0.0f, 0.0f, 4u);
    geo_zone_extent_t e;
    geo_zone_extent(&m, 0u, 1u, &e);
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 11.25f, e.az_max_deg - e.az_min_deg);
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, e.az_min_deg + 5.625f, e.az_center_deg);
}

/* ---- MAP-02: overlap of neighbouring sensors ---- */

void test_MAP02_overlap_of_neighbouring_sensors(void)
{
    const float theta[3] = {25.0f, 30.0f, 40.0f};
    for (unsigned k = 0; k < 3u; k++) {
        const geo_mount_t c = mount(0.0f, 0.0f, 0.0f, 4u);
        const geo_mount_t r = mount(theta[k], 0.0f, 0.0f, 4u);
        const geo_mount_t l = mount(-theta[k], 0.0f, 0.0f, 4u);
        geo_zone_extent_t c_right;
        geo_zone_extent_t r_left;
        geo_zone_extent_t c_left;
        geo_zone_extent_t l_right;
        geo_zone_extent(&c, 0u, 3u, &c_right);
        geo_zone_extent(&r, 0u, 0u, &r_left);
        geo_zone_extent(&c, 0u, 0u, &c_left);
        geo_zone_extent(&l, 0u, 3u, &l_right);
        /* Overlap width = FoV - theta = 45 - theta, symmetric on both sides. */
        TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 45.0f - theta[k], c_right.az_max_deg - r_left.az_min_deg);
        TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, 45.0f - theta[k], l_right.az_max_deg - c_left.az_min_deg);
    }
}

/* ---- MAP-02: vehicle pitch and roll +-15 deg, heights 0.2 / 1 / 7 m ---- */

void test_MAP02_vehicle_pitch_15_both_signs(void)
{
    const geo_mount_t m = mount(0.0f, 0.0f, 0.0f, 4u);
    const geo_vec3_t p_s = {2.0f, 0.0f, 0.0f};
    const float th = geo_deg_to_rad(15.0f);
    /* Nose up: a forward point is above the sensor. */
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f + 2.0f * sind(15.0f),
                             geo_point_height(&m, p_s, th, 0.0f, 1.0f));
    /* Nose down: it is closer to the ground. */
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f - 2.0f * sind(15.0f),
                             geo_point_height(&m, p_s, -th, 0.0f, 1.0f));
    const geo_vec3_t e = {2.0f * cosd(15.0f), 0.0f, -2.0f * sind(15.0f)};
    assert_vec(e, geo_body_to_level(p_s, th, 0.0f), TOL);
}

void test_MAP02_vehicle_roll_15_both_signs(void)
{
    const geo_mount_t r_sensor = mount(90.0f, 0.0f, 0.0f, 4u); /* looks to the right */
    const geo_vec3_t p_s = {2.0f, 0.0f, 0.0f};
    const float ph = geo_deg_to_rad(15.0f);
    /* Right side down: a point on the right is lower. */
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f - 2.0f * sind(15.0f),
                             geo_point_height(&r_sensor, p_s, 0.0f, ph, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f + 2.0f * sind(15.0f),
                             geo_point_height(&r_sensor, p_s, 0.0f, -ph, 1.0f));
    /* Roll does not change the height of a point straight ahead. */
    const geo_mount_t c = mount(0.0f, 0.0f, 0.0f, 4u);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, geo_point_height(&c, p_s, 0.0f, ph, 1.0f));
}

/* The bottom row hits flat ground where h_p = 0; at 0.2 m: 0.66 m (row 3) and 2.0 m (row 2). */
void test_MAP02_ground_hit_heights_02_1_7(void)
{
    const float heights[3] = {0.2f, 1.0f, 7.0f};
    const geo_mount_t m = mount(0.0f, 0.0f, 0.0f, 4u);
    for (unsigned k = 0; k < 3u; k++) {
        const float h = heights[k];
        for (uint8_t i = 2u; i < 4u; i++) {
            const float b = geo_zone_beta_deg(4u, i);
            const float r_ground = h / sind(-b);
            const geo_vec3_t p = geo_zone_point_sensor(&m, i, 1u, r_ground);
            TEST_ASSERT_FLOAT_WITHIN(1e-3f * h, 0.0f, geo_point_height(&m, p, 0.0f, 0.0f, h));
            float d = 0.0f;
            float az = 0.0f;
            geo_zone_target(&m, i, 1u, r_ground, &d, &az);
            TEST_ASSERT_FLOAT_WITHIN(1e-3f * h, h / tand(-b), d);
        }
    }
    float d = 0.0f;
    float az = 0.0f;
    geo_zone_target(&m, 3u, 1u, 0.2f / sind(16.875f), &d, &az);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.66f, d);
    geo_zone_target(&m, 2u, 1u, 0.2f / sind(5.625f), &d, &az);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 2.03f, d);
}

/* At 7 m with the nose 15 deg down the ground hit of the bottom row moves closer. */
void test_MAP02_ground_hit_at_7m_nose_down(void)
{
    const geo_mount_t m = mount(0.0f, 0.0f, 0.0f, 4u);
    const float b = geo_zone_beta_deg(4u, 3u) - 15.0f;
    const float r_ground = 7.0f / sind(-b);
    const geo_vec3_t p = geo_zone_point_sensor(&m, 3u, 1u, r_ground);
    /* Centre column 1 has a small azimuth, so the hit is near but not exactly at zero height. */
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f,
                             geo_point_height(&m, p, geo_deg_to_rad(-15.0f), 0.0f, 7.0f));
}

/* ---- MAP-02: mirroring (ORIENT_TRANSFORM) ---- */

void test_MAP02_orient_identity_and_mirror(void)
{
    uint8_t i = 0u;
    uint8_t j = 0u;
    geo_orient_apply(GEO_ORIENT_IDENTITY, 4u, 1u, 0u, &i, &j);
    TEST_ASSERT_EQUAL_UINT8(1u, i);
    TEST_ASSERT_EQUAL_UINT8(0u, j);
    geo_orient_apply(GEO_ORIENT_MIRROR, 4u, 1u, 0u, &i, &j);
    TEST_ASSERT_EQUAL_UINT8(1u, i);
    TEST_ASSERT_EQUAL_UINT8(3u, j);

    /* A target in raw column 0 is on the left without mirroring and on the right with it. */
    geo_mount_t m;
    const geo_vec3_t t = {0.0f, 0.0f, 0.0f};
    geo_mount_init(&m, 0.0f, 0.0f, 0.0f, t, 4u, GEO_ORIENT_MIRROR);
    uint8_t zi = 0u;
    uint8_t zj = 0u;
    geo_zone_from_raw(&m, 0u, &zi, &zj); /* raw zone 0: row 0, column 0 */
    float d = 0.0f;
    float az = 0.0f;
    geo_zone_target(&m, zi, zj, 1.0f, &d, &az);
    TEST_ASSERT_TRUE(az > 0.0f);
    geo_mount_init(&m, 0.0f, 0.0f, 0.0f, t, 4u, GEO_ORIENT_IDENTITY);
    geo_zone_from_raw(&m, 0u, &zi, &zj);
    geo_zone_target(&m, zi, zj, 1.0f, &d, &az);
    TEST_ASSERT_TRUE(az < 0.0f);
}

void test_MAP02_orient_rotations(void)
{
    uint8_t i = 0u;
    uint8_t j = 0u;
    /* 90 deg clockwise: top-left goes to top-right. */
    geo_orient_apply(GEO_ORIENT_ROT90, 4u, 0u, 0u, &i, &j);
    TEST_ASSERT_EQUAL_UINT8(0u, i);
    TEST_ASSERT_EQUAL_UINT8(3u, j);
    geo_orient_apply(GEO_ORIENT_ROT180, 4u, 0u, 0u, &i, &j);
    TEST_ASSERT_EQUAL_UINT8(3u, i);
    TEST_ASSERT_EQUAL_UINT8(3u, j);
    geo_orient_apply(GEO_ORIENT_ROT270, 4u, 0u, 0u, &i, &j);
    TEST_ASSERT_EQUAL_UINT8(3u, i);
    TEST_ASSERT_EQUAL_UINT8(0u, j);
    /* Mirror then 90 deg: top-left -> top-right (mirror) -> bottom-right (rot). */
    geo_orient_apply(GEO_ORIENT_MIRROR | GEO_ORIENT_ROT90, 4u, 0u, 0u, &i, &j);
    TEST_ASSERT_EQUAL_UINT8(3u, i);
    TEST_ASSERT_EQUAL_UINT8(3u, j);
}

/* ---- MAP-02: mounting offset XYZ ---- */

void test_MAP02_mount_offset_xyz(void)
{
    geo_mount_t m;
    const geo_vec3_t t = {0.10f, 0.05f, -0.02f};
    geo_mount_init(&m, 0.0f, 0.0f, 0.0f, t, 4u, GEO_ORIENT_IDENTITY);
    const geo_vec3_t p_s = {1.0f, 0.0f, 0.0f};
    const geo_vec3_t e = {1.10f, 0.05f, -0.02f};
    assert_vec(e, geo_sensor_to_body(&m, p_s), TOL);
    TEST_ASSERT_FLOAT_WITHIN(TOL, sqrtf(1.10f * 1.10f + 0.05f * 0.05f), geo_horiz_dist(e));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DEG, geo_rad_to_deg(atan2f(0.05f, 1.10f)), geo_azimuth_deg(e));

    /* Offset is applied after the rotation (in the body frame). */
    geo_mount_init(&m, 90.0f, 0.0f, 0.0f, t, 4u, GEO_ORIENT_IDENTITY);
    const geo_vec3_t e90 = {0.10f, 1.05f, -0.02f};
    assert_vec(e90, geo_sensor_to_body(&m, p_s), TOL);
    /* MAP-14a: heights refer to the body centre, so t_m enters: z_L(p_B) = -0.02. */
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.52f, geo_point_height(&m, p_s, 0.0f, 0.0f, 0.5f));
}

/* ---- MAP-02: invertibility ---- */

void test_MAP02_rotation_is_orthonormal_and_invertible(void)
{
    geo_mount_t m;
    const geo_vec3_t t = {0.03f, -0.04f, 0.01f};
    geo_mount_init(&m, -30.0f, 4.0f, -3.0f, t, 4u, GEO_ORIENT_IDENTITY);
    const geo_mat3_t rt = geo_mat_transpose(m.r_bs);
    const geo_mat3_t id = geo_mat_mul(rt, m.r_bs);
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            TEST_ASSERT_FLOAT_WITHIN(1e-5f, r == c ? 1.0f : 0.0f, id.m[r][c]);
        }
    }
    const geo_vec3_t p_s = geo_dir_sensor(7.0f, -13.0f);
    const geo_vec3_t p_b = geo_sensor_to_body(&m, p_s);
    assert_vec(p_s, geo_body_to_sensor(&m, p_b), 1e-5f);
    /* Body -> level -> body with the transposed attitude rotation. */
    const geo_vec3_t p_l = geo_body_to_level(p_b, 0.2f, -0.1f);
    const geo_mat3_t r_lb = geo_mat_mul(geo_rot_y(0.2f), geo_rot_x(-0.1f));
    assert_vec(p_b, geo_mat_apply(geo_mat_transpose(r_lb), p_l), 1e-5f);
    /* d and psi reconstruct the horizontal part of p_B. */
    const float d = geo_horiz_dist(p_b);
    const float az = geo_deg_to_rad(geo_azimuth_deg(p_b));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, p_b.x, d * cosf(az));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, p_b.y, d * sinf(az));
}

void test_MAP02_orient_transform_round_trip_all_codes(void)
{
    const uint8_t sizes[2] = {4u, 8u};
    for (unsigned s = 0; s < 2u; s++) {
        const uint8_t n = sizes[s];
        for (uint8_t code = 0; code < GEO_ORIENT_COUNT; code++) {
            uint8_t seen[TOF_MAX_ZONES] = {0};
            for (uint8_t r = 0; r < n; r++) {
                for (uint8_t c = 0; c < n; c++) {
                    uint8_t i = 0u;
                    uint8_t j = 0u;
                    uint8_t r2 = 0u;
                    uint8_t c2 = 0u;
                    geo_orient_apply(code, n, r, c, &i, &j);
                    TEST_ASSERT_TRUE(i < n && j < n);
                    seen[i * n + j]++;
                    geo_orient_inverse(code, n, i, j, &r2, &c2);
                    TEST_ASSERT_EQUAL_UINT8(r, r2);
                    TEST_ASSERT_EQUAL_UINT8(c, c2);
                }
            }
            for (unsigned z = 0; z < (unsigned)n * n; z++) {
                TEST_ASSERT_EQUAL_UINT8(1u, seen[z]); /* bijection */
            }
        }
    }
}

void test_MAP02_zone_from_raw_out_of_range_code_is_identity(void)
{
    geo_mount_t m;
    const geo_vec3_t t = {0.0f, 0.0f, 0.0f};
    geo_mount_init(&m, 0.0f, 0.0f, 0.0f, t, 8u, 99u);
    TEST_ASSERT_EQUAL_UINT8(GEO_ORIENT_IDENTITY, m.orient);
    uint8_t i = 0u;
    uint8_t j = 0u;
    geo_zone_from_raw(&m, 13u, &i, &j);
    TEST_ASSERT_EQUAL_UINT8(1u, i);
    TEST_ASSERT_EQUAL_UINT8(5u, j);
}

/* Mount pitch / roll at +-15 deg against hand values. */
void test_MAP02_mount_pitch_roll_15(void)
{
    const geo_vec3_t fwd = {1.0f, 0.0f, 0.0f};
    const geo_vec3_t ey = {0.0f, 1.0f, 0.0f};
    const geo_mount_t up = mount(0.0f, 15.0f, 0.0f, 4u);
    const geo_vec3_t e_up = {0.965926f, 0.0f, -0.258819f};
    assert_vec(e_up, geo_sensor_to_body(&up, fwd), TOL);
    const geo_mount_t dn = mount(0.0f, -15.0f, 0.0f, 4u);
    const geo_vec3_t e_dn = {0.965926f, 0.0f, 0.258819f};
    assert_vec(e_dn, geo_sensor_to_body(&dn, fwd), TOL);
    const geo_mount_t rp = mount(0.0f, 0.0f, 15.0f, 4u);
    const geo_vec3_t e_rp = {0.0f, 0.965926f, 0.258819f};
    assert_vec(e_rp, geo_sensor_to_body(&rp, ey), TOL);
    const geo_mount_t rn = mount(0.0f, 0.0f, -15.0f, 4u);
    const geo_vec3_t e_rn = {0.0f, 0.965926f, -0.258819f};
    assert_vec(e_rn, geo_sensor_to_body(&rn, ey), TOL);
}

/* ZYX against independent hand values. */
void test_MAP02_zyx_hand_values(void)
{
    const geo_vec3_t ey = {0.0f, 1.0f, 0.0f};
    const geo_vec3_t ex = {1.0f, 0.0f, 0.0f};
    /* Roll 90 first: y_S -> down; yaw 90 keeps it down. */
    const geo_mount_t a = mount(90.0f, 0.0f, 90.0f, 4u);
    const geo_vec3_t down = {0.0f, 0.0f, 1.0f};
    assert_vec(down, geo_sensor_to_body(&a, ey), TOL);
    /* Pitch 90 first: x_S -> up; yaw 90 keeps it up. In XYZ order it would point right. */
    const geo_mount_t b = mount(90.0f, 90.0f, 0.0f, 4u);
    const geo_vec3_t up = {0.0f, 0.0f, -1.0f};
    assert_vec(up, geo_sensor_to_body(&b, ex), TOL);
    /* [cos30 cos20, sin30 cos20, -sin20] */
    const geo_mount_t c = mount(30.0f, 20.0f, 0.0f, 4u);
    const geo_vec3_t e = {0.813798f, 0.469846f, -0.342020f};
    assert_vec(e, geo_sensor_to_body(&c, ex), TOL);
}

/* Vehicle pitch and roll together: p_L = Ry(theta) Rx(phi) p_B, hand value. */
void test_MAP02_vehicle_pitch_and_roll_combined(void)
{
    const geo_vec3_t p = {1.0f, 1.0f, 0.0f};
    /* Rx(15) p = [1, c, s]; Ry(15): x' = c + s * s, z' = -s + c * s */
    const float c = 0.965926f;
    const float s = 0.258819f;
    const geo_vec3_t e = {c + s * s, c, -s + c * s};
    assert_vec(e, geo_body_to_level(p, geo_deg_to_rad(15.0f), geo_deg_to_rad(15.0f)), TOL);
    const geo_vec3_t e2 = {c + s * s, c, s - c * s};
    assert_vec(e2, geo_body_to_level(p, geo_deg_to_rad(-15.0f), geo_deg_to_rad(-15.0f)), TOL);
}

/* Heights 0.2 / 1 / 7 m with vehicle pitch and roll -15 / 0 / +15 deg in every combination. */
void test_MAP02_heights_with_attitude_15(void)
{
    const float heights[3] = {0.2f, 1.0f, 7.0f};
    const float att[3] = {-15.0f, 0.0f, 15.0f};
    const geo_mount_t m = mount(30.0f, 0.0f, 0.0f, 4u);
    unsigned checked = 0u;
    for (unsigned k = 0; k < 3u; k++) {
        for (unsigned a = 0; a < 3u; a++) {
            for (unsigned b = 0; b < 3u; b++) {
                const float th = geo_deg_to_rad(att[a]);
                const float ph = geo_deg_to_rad(att[b]);
                const geo_vec3_t u = geo_zone_point_sensor(&m, 3u, 1u, 1.0f);
                const float down = geo_body_to_level(geo_mat_apply(m.r_bs, u), th, ph).z;
                if (down <= 0.01f) {
                    continue; /* ray does not reach the ground */
                }
                const float r = heights[k] / down;
                const float tol = 1e-4f * heights[k] + 1e-5f;
                const geo_vec3_t p = geo_zone_point_sensor(&m, 3u, 1u, r);
                TEST_ASSERT_FLOAT_WITHIN(tol, 0.0f, geo_point_height(&m, p, th, ph, heights[k]));
                const geo_vec3_t p2 = geo_zone_point_sensor(&m, 3u, 1u, 0.5f * r);
                TEST_ASSERT_FLOAT_WITHIN(tol, 0.5f * heights[k],
                                         geo_point_height(&m, p2, th, ph, heights[k]));
                checked++;
            }
        }
    }
    TEST_ASSERT_TRUE(checked >= 18u);
}

/* MAP-14a: body-centre height from the sensor optics height with MOUNT_XYZ_M. */
void test_MAP14a_body_height_from_sensor(void)
{
    geo_mount_t m;
    const geo_vec3_t t = {0.10f, 0.0f, 0.05f}; /* sensor 5 cm below the body centre */
    geo_mount_init(&m, 0.0f, 0.0f, 0.0f, t, 4u, GEO_ORIENT_IDENTITY);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.25f, geo_body_height_from_sensor(&m, 0.20f, 0.0f, 0.0f));
    /* Nose up 10 deg: the forward offset rises: z_L(t) = -0.1 sin10 + 0.05 cos10. */
    const float th = geo_deg_to_rad(10.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.20f - 0.10f * sind(10.0f) + 0.05f * cosd(10.0f),
                             geo_body_height_from_sensor(&m, 0.20f, th, 0.0f));
}

/* MAP-14a: body-centre height from a downward rangefinder with RF_XYZ_M. */
void test_MAP14a_body_height_from_rangefinder(void)
{
    const geo_vec3_t t_rf = {0.0f, 0.0f, 0.03f};
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.03f, geo_body_height_from_rf(1.0f, t_rf, 0.0f, 0.0f));
    const float th = geo_deg_to_rad(10.0f);
    const float ph = geo_deg_to_rad(5.0f);
    const float e = 1.0f * cosd(10.0f) * cosd(5.0f) + 0.03f * cosd(10.0f) * cosd(5.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, e, geo_body_height_from_rf(1.0f, t_rf, th, ph));
}

/* MAP-14a: with offsets and attitude, the ground hit of a ray has h_p = 0 when h_body comes from
 * the optics height of the same sensor. */
void test_MAP14a_ground_hit_with_offset_and_attitude(void)
{
    geo_mount_t m;
    const geo_vec3_t t = {0.12f, -0.04f, 0.06f};
    geo_mount_init(&m, -30.0f, 0.0f, 0.0f, t, 4u, GEO_ORIENT_IDENTITY);
    const float th = geo_deg_to_rad(-5.0f);
    const float ph = geo_deg_to_rad(3.0f);
    const float h_opt = 0.2f;
    const float h_body = geo_body_height_from_sensor(&m, h_opt, th, ph);
    const geo_vec3_t u = geo_zone_point_sensor(&m, 3u, 1u, 1.0f);
    const float down = geo_body_to_level(geo_mat_apply(m.r_bs, u), th, ph).z;
    TEST_ASSERT_TRUE(down > 0.0f);
    const geo_vec3_t p = geo_zone_point_sensor(&m, 3u, 1u, h_opt / down);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, geo_point_height(&m, p, th, ph, h_body));
}

/* MAP-05a: the extent at range equals the direction extent when t_m = 0 and converges to it far away. */
void test_MAP05a_extent_at_range(void)
{
    geo_mount_t m;
    const geo_vec3_t t0 = {0.0f, 0.0f, 0.0f};
    geo_mount_init(&m, 30.0f, 4.0f, -3.0f, t0, 4u, GEO_ORIENT_IDENTITY);
    geo_zone_extent_t a;
    geo_zone_extent_t b;
    geo_zone_extent(&m, 2u, 1u, &a);
    geo_zone_extent_at_range(&m, 2u, 1u, 0.5f, &b);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, a.az_min_deg, b.az_min_deg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, a.az_max_deg, b.az_max_deg);

    const geo_vec3_t ty = {0.0f, 0.10f, 0.0f};
    geo_mount_init(&m, 0.0f, 0.0f, 0.0f, ty, 4u, GEO_ORIENT_IDENTITY);
    geo_zone_extent(&m, 1u, 1u, &a);
    geo_zone_extent_at_range(&m, 1u, 1u, 0.3f, &b);
    /* Centre ray alpha -5.625: (0.2986, -0.0294 + 0.1) -> +13.4 deg from the body centre. */
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 13.4f, b.az_center_deg);
    TEST_ASSERT_TRUE(b.az_min_deg > a.az_max_deg);
    geo_zone_extent_at_range(&m, 1u, 1u, 1000.0f, &b);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, a.az_center_deg, b.az_center_deg);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_MAP02_deg_to_rad);
    RUN_TEST(test_MAP02_wrap_keeps_range_0_360);
    RUN_TEST(test_MAP02_wrap_negative_angles);
    RUN_TEST(test_MAP02_wrap_tiny_negative_is_below_360);
    RUN_TEST(test_MAP02_wrap_180);
    RUN_TEST(test_MAP01_zone_angles_4x4_and_8x8);
    RUN_TEST(test_MAP01_sensor_frame_signs);
    RUN_TEST(test_MAP01_elementary_rotations);
    RUN_TEST(test_MAP02_mount_yaw_positive_is_right);
    RUN_TEST(test_MAP02_mount_yaw_negative_is_left);
    RUN_TEST(test_MAP02_mount_pitch_positive_is_nose_up);
    RUN_TEST(test_MAP02_mount_roll_positive_is_right_side_down);
    RUN_TEST(test_MAP02_zyx_order);
    RUN_TEST(test_MAP02_three_sensors_zone_azimuths);
    RUN_TEST(test_MAP02_three_sensors_extents);
    RUN_TEST(test_MAP02_extent_contains_zone_directions_when_tilted);
    RUN_TEST(test_MAP02_extent_across_rear_seam);
    RUN_TEST(test_MAP02_overlap_of_neighbouring_sensors);
    RUN_TEST(test_MAP02_vehicle_pitch_15_both_signs);
    RUN_TEST(test_MAP02_vehicle_roll_15_both_signs);
    RUN_TEST(test_MAP02_ground_hit_heights_02_1_7);
    RUN_TEST(test_MAP02_ground_hit_at_7m_nose_down);
    RUN_TEST(test_MAP02_orient_identity_and_mirror);
    RUN_TEST(test_MAP02_orient_rotations);
    RUN_TEST(test_MAP02_mount_offset_xyz);
    RUN_TEST(test_MAP02_rotation_is_orthonormal_and_invertible);
    RUN_TEST(test_MAP02_orient_transform_round_trip_all_codes);
    RUN_TEST(test_MAP02_zone_from_raw_out_of_range_code_is_identity);
    RUN_TEST(test_MAP02_mount_pitch_roll_15);
    RUN_TEST(test_MAP02_zyx_hand_values);
    RUN_TEST(test_MAP02_vehicle_pitch_and_roll_combined);
    RUN_TEST(test_MAP02_heights_with_attitude_15);
    RUN_TEST(test_MAP14a_body_height_from_sensor);
    RUN_TEST(test_MAP14a_body_height_from_rangefinder);
    RUN_TEST(test_MAP14a_ground_hit_with_offset_and_attitude);
    RUN_TEST(test_MAP05a_extent_at_range);
    return UNITY_END();
}
