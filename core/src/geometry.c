#include "core/geometry.h"

#include <math.h>

/* REQ: MAP-02 (helper) */
float geo_deg_to_rad(float deg)
{
    return deg * GEO_DEG_TO_RAD_F;
}

/* REQ: MAP-02 (helper) */
float geo_rad_to_deg(float rad)
{
    return rad / GEO_DEG_TO_RAD_F;
}

/* REQ: MAP-02 (helper) */
float geo_wrap_deg_360(float deg)
{
    float r = fmodf(deg, 360.0f);
    if (r < 0.0f) {
        r += 360.0f;
    }
    /* A tiny negative input rounds up to exactly 360 after the addition. */
    if (r >= 360.0f) {
        r = 0.0f;
    }
    return r;
}

/* REQ: MAP-02 (helper) */
float geo_wrap_deg_180(float deg)
{
    return geo_wrap_deg_360(deg + 180.0f) - 180.0f;
}

/* REQ: MAP-01 */
geo_mat3_t geo_rot_x(float rad)
{
    const float c = cosf(rad);
    const float s = sinf(rad);
    const geo_mat3_t r = {{{1.0f, 0.0f, 0.0f}, {0.0f, c, -s}, {0.0f, s, c}}};
    return r;
}

/* REQ: MAP-01 (Ry(theta) [1, 0, 0] = [cos theta, 0, -sin theta]) */
geo_mat3_t geo_rot_y(float rad)
{
    const float c = cosf(rad);
    const float s = sinf(rad);
    const geo_mat3_t r = {{{c, 0.0f, s}, {0.0f, 1.0f, 0.0f}, {-s, 0.0f, c}}};
    return r;
}

/* REQ: MAP-01 */
geo_mat3_t geo_rot_z(float rad)
{
    const float c = cosf(rad);
    const float s = sinf(rad);
    const geo_mat3_t r = {{{c, -s, 0.0f}, {s, c, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    return r;
}

/* REQ: MAP-02 (helper) */
geo_mat3_t geo_mat_mul(geo_mat3_t a, geo_mat3_t b)
{
    geo_mat3_t r;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
        }
    }
    return r;
}

/* REQ: MAP-02 (helper) */
geo_mat3_t geo_mat_transpose(geo_mat3_t a)
{
    geo_mat3_t r;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            r.m[i][j] = a.m[j][i];
        }
    }
    return r;
}

/* REQ: MAP-02 (helper) */
geo_vec3_t geo_mat_apply(geo_mat3_t a, geo_vec3_t v)
{
    geo_vec3_t r;
    r.x = a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z;
    r.y = a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z;
    r.z = a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z;
    return r;
}

/* REQ: MAP-02 */
float geo_zone_width_deg(uint8_t n)
{
    return TOF_SENSOR_FOV_DEG / (float)n;
}

/* REQ: MAP-02 (alpha_j = (j - (N-1)/2) * 45/N) */
float geo_zone_alpha_deg(uint8_t n, uint8_t j)
{
    return ((float)j - (float)(n - 1u) / 2.0f) * geo_zone_width_deg(n);
}

/* REQ: MAP-02 (beta_i = ((N-1)/2 - i) * 45/N) */
float geo_zone_beta_deg(uint8_t n, uint8_t i)
{
    return ((float)(n - 1u) / 2.0f - (float)i) * geo_zone_width_deg(n);
}

/* REQ: MAP-01, MAP-02 */
geo_vec3_t geo_dir_sensor(float alpha_deg, float beta_deg)
{
    const float a = geo_deg_to_rad(alpha_deg);
    const float b = geo_deg_to_rad(beta_deg);
    const geo_vec3_t v = {cosf(b) * cosf(a), cosf(b) * sinf(a), -sinf(b)};
    return v;
}

/* REQ: MAP-02, SNS-12 */
void geo_orient_apply(uint8_t code, uint8_t n, uint8_t raw_row, uint8_t raw_col, uint8_t *i, uint8_t *j)
{
    uint8_t r = raw_row;
    uint8_t c = raw_col;
    if ((code & GEO_ORIENT_MIRROR) != 0u) {
        c = (uint8_t)(n - 1u - c);
    }
    for (uint8_t k = 0; k < (code & 3u); k++) {
        /* 90 deg clockwise: (r, c) -> (c, N-1-r). */
        const uint8_t nr = c;
        c = (uint8_t)(n - 1u - r);
        r = nr;
    }
    *i = r;
    *j = c;
}

/* REQ: MAP-02, SNS-12 */
void geo_orient_inverse(uint8_t code, uint8_t n, uint8_t i, uint8_t j, uint8_t *raw_row, uint8_t *raw_col)
{
    uint8_t r = i;
    uint8_t c = j;
    for (uint8_t k = 0; k < (code & 3u); k++) {
        /* 90 deg counter-clockwise: (r, c) -> (N-1-c, r). */
        const uint8_t nr = (uint8_t)(n - 1u - c);
        c = r;
        r = nr;
    }
    if ((code & GEO_ORIENT_MIRROR) != 0u) {
        c = (uint8_t)(n - 1u - c);
    }
    *raw_row = r;
    *raw_col = c;
}

/* REQ: MAP-02 (ZYX: Rz(psi_m) Ry(theta_m) Rx(phi_m)) */
void geo_mount_init(geo_mount_t *m, float yaw_deg, float pitch_deg, float roll_deg, geo_vec3_t t_m,
                    uint8_t n, uint8_t orient)
{
    m->r_bs = geo_mat_mul(geo_rot_z(geo_deg_to_rad(yaw_deg)),
                          geo_mat_mul(geo_rot_y(geo_deg_to_rad(pitch_deg)),
                                      geo_rot_x(geo_deg_to_rad(roll_deg))));
    m->t_m = t_m;
    m->n = n;
    m->orient = (orient < GEO_ORIENT_COUNT) ? orient : GEO_ORIENT_IDENTITY;
}

/* REQ: MAP-02, SNS-12 */
void geo_zone_from_raw(const geo_mount_t *m, uint8_t raw_zone, uint8_t *i, uint8_t *j)
{
    geo_orient_apply(m->orient, m->n, (uint8_t)(raw_zone / m->n), (uint8_t)(raw_zone % m->n), i, j);
}

/* REQ: MAP-02 */
geo_vec3_t geo_sensor_to_body(const geo_mount_t *m, geo_vec3_t p_s)
{
    geo_vec3_t p = geo_mat_apply(m->r_bs, p_s);
    p.x += m->t_m.x;
    p.y += m->t_m.y;
    p.z += m->t_m.z;
    return p;
}

/* REQ: MAP-02 */
geo_vec3_t geo_body_to_sensor(const geo_mount_t *m, geo_vec3_t p_b)
{
    const geo_vec3_t d = {p_b.x - m->t_m.x, p_b.y - m->t_m.y, p_b.z - m->t_m.z};
    return geo_mat_apply(geo_mat_transpose(m->r_bs), d);
}

/* REQ: MAP-02 (p_L = Ry(theta) Rx(phi) p_B) */
geo_vec3_t geo_body_to_level(geo_vec3_t p_b, float pitch_rad, float roll_rad)
{
    return geo_mat_apply(geo_mat_mul(geo_rot_y(pitch_rad), geo_rot_x(roll_rad)), p_b);
}

/* REQ: MAP-04 */
float geo_horiz_dist(geo_vec3_t p_b)
{
    return sqrtf(p_b.x * p_b.x + p_b.y * p_b.y);
}

/* REQ: MAP-04 */
float geo_azimuth_deg(geo_vec3_t p_b)
{
    return geo_rad_to_deg(atan2f(p_b.y, p_b.x));
}

/* REQ: MAP-02 */
geo_vec3_t geo_zone_point_sensor(const geo_mount_t *m, uint8_t i, uint8_t j, float r_m)
{
    const geo_vec3_t u = geo_dir_sensor(geo_zone_alpha_deg(m->n, j), geo_zone_beta_deg(m->n, i));
    const geo_vec3_t p = {u.x * r_m, u.y * r_m, u.z * r_m};
    return p;
}

/* REQ: MAP-04 */
void geo_zone_target(const geo_mount_t *m, uint8_t i, uint8_t j, float r_m, float *d_m, float *az_deg)
{
    const geo_vec3_t p_b = geo_sensor_to_body(m, geo_zone_point_sensor(m, i, j, r_m));
    *d_m = geo_horiz_dist(p_b);
    *az_deg = geo_azimuth_deg(p_b);
}

/* REQ: MAP-02 (zone extent is stored). Direction-only: t_m parallax is not included (DEV-05).
 * The azimuth range is sampled on a 5 x 5 grid over the zone, which is exact for an untilted
 * mount and conservative to well below 0.05 deg for the +-15 deg mounting errors allowed. */
void geo_zone_extent(const geo_mount_t *m, uint8_t i, uint8_t j, geo_zone_extent_t *out)
{
    const float w = geo_zone_width_deg(m->n);
    const float a_c = geo_zone_alpha_deg(m->n, j);
    const float b_c = geo_zone_beta_deg(m->n, i);
    const float az_c = geo_azimuth_deg(geo_mat_apply(m->r_bs, geo_dir_sensor(a_c, b_c)));
    float lo = 0.0f;
    float hi = 0.0f;
    for (int ka = 0; ka <= 4; ka++) {
        for (int kb = 0; kb <= 4; kb++) {
            const float a = a_c - w / 2.0f + w * (float)ka / 4.0f;
            const float b = b_c - w / 2.0f + w * (float)kb / 4.0f;
            const float az = geo_azimuth_deg(geo_mat_apply(m->r_bs, geo_dir_sensor(a, b)));
            const float d = geo_wrap_deg_180(az - az_c);
            lo = (d < lo) ? d : lo;
            hi = (d > hi) ? d : hi;
        }
    }
    out->az_center_deg = az_c;
    out->az_min_deg = az_c + lo;
    out->az_max_deg = az_c + hi;
    out->alpha_min_deg = a_c - w / 2.0f;
    out->alpha_max_deg = a_c + w / 2.0f;
    out->beta_min_deg = b_c - w / 2.0f;
    out->beta_max_deg = b_c + w / 2.0f;
}

/* REQ: MAP-02 (h_p = h_tof - z_L) */
float geo_point_height(const geo_mount_t *m, geo_vec3_t p_s, float pitch_rad, float roll_rad, float h_tof_m)
{
    const geo_vec3_t p_l = geo_body_to_level(geo_mat_apply(m->r_bs, p_s), pitch_rad, roll_rad);
    return h_tof_m - p_l.z;
}
