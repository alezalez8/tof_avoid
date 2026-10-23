#ifndef CORE_GEOMETRY_H
#define CORE_GEOMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "core/config.h"

/* Frames and zone geometry of TZ 5.1 (MAP-01, MAP-02, MAP-04).
 * Sensor S: x along the optical axis, y right, z down. Body B: FRD. Level L: B without roll and pitch.
 * Azimuth psi positive to the right, elevation up, pitch nose up, roll right side down. */

#define GEO_PI_F 3.14159265f
#define GEO_DEG_TO_RAD_F (GEO_PI_F / 180.0f)

typedef struct {
    float x;
    float y;
    float z;
} geo_vec3_t;

typedef struct {
    float m[3][3];
} geo_mat3_t;

/* ORIENT_TRANSFORM codes (SNS-12): bit 2 mirrors columns, bits 0..1 rotate clockwise by 90 deg
 * steps; mirroring is applied first. */
#define GEO_ORIENT_IDENTITY 0u
#define GEO_ORIENT_ROT90 1u
#define GEO_ORIENT_ROT180 2u
#define GEO_ORIENT_ROT270 3u
#define GEO_ORIENT_MIRROR 4u
#define GEO_ORIENT_COUNT 8u

/* Mounting of one sensor: body <- sensor rotation, offset and matrix layout. */
typedef struct {
    geo_mat3_t r_bs; /* Rz(psi_m) Ry(theta_m) Rx(phi_m) */
    geo_vec3_t t_m;  /* sensor position in the body frame, m */
    uint8_t n;       /* matrix side: 4 or 8 */
    uint8_t orient;  /* ORIENT_TRANSFORM code */
} geo_mount_t;

/* Angular extent of a zone: azimuth in the body frame (az_min <= az_center <= az_max, contiguous,
 * may leave [-180, 180) at the rear seam) and the alpha/beta box in the sensor frame. */
typedef struct {
    float az_min_deg;
    float az_max_deg;
    float az_center_deg;
    float alpha_min_deg;
    float alpha_max_deg;
    float beta_min_deg;
    float beta_max_deg;
} geo_zone_extent_t;

float geo_deg_to_rad(float deg);
float geo_rad_to_deg(float rad);

/* Wraps an angle to [0, 360) degrees. */
float geo_wrap_deg_360(float deg);

/* Wraps an angle to [-180, 180) degrees. */
float geo_wrap_deg_180(float deg);

geo_mat3_t geo_rot_x(float rad);
geo_mat3_t geo_rot_y(float rad);
geo_mat3_t geo_rot_z(float rad);
geo_mat3_t geo_mat_mul(geo_mat3_t a, geo_mat3_t b);
geo_mat3_t geo_mat_transpose(geo_mat3_t a);
geo_vec3_t geo_mat_apply(geo_mat3_t a, geo_vec3_t v);

/* Zone angles of TZ 5.1 for an N x N matrix after ORIENT_TRANSFORM; i = 0 is the top row. */
float geo_zone_alpha_deg(uint8_t n, uint8_t j);
float geo_zone_beta_deg(uint8_t n, uint8_t i);
float geo_zone_width_deg(uint8_t n);

/* Unit vector [cos b cos a, cos b sin a, -sin b] in the sensor frame. */
geo_vec3_t geo_dir_sensor(float alpha_deg, float beta_deg);

/* ORIENT_TRANSFORM: raw ULD (row, column) -> (i, j) and back. */
void geo_orient_apply(uint8_t code, uint8_t n, uint8_t raw_row, uint8_t raw_col, uint8_t *i, uint8_t *j);
void geo_orient_inverse(uint8_t code, uint8_t n, uint8_t i, uint8_t j, uint8_t *raw_row, uint8_t *raw_col);

/* Angles in degrees; an unknown orient code falls back to identity. */
void geo_mount_init(geo_mount_t *m, float yaw_deg, float pitch_deg, float roll_deg, geo_vec3_t t_m,
                    uint8_t n, uint8_t orient);

/* Raw ULD zone index (row-major) -> (i, j) after the mount's ORIENT_TRANSFORM. */
void geo_zone_from_raw(const geo_mount_t *m, uint8_t raw_zone, uint8_t *i, uint8_t *j);

/* p_B = R p_S + t_m and its inverse. */
geo_vec3_t geo_sensor_to_body(const geo_mount_t *m, geo_vec3_t p_s);
geo_vec3_t geo_body_to_sensor(const geo_mount_t *m, geo_vec3_t p_b);

/* p_L = Ry(theta) Rx(phi) p_B. */
geo_vec3_t geo_body_to_level(geo_vec3_t p_b, float pitch_rad, float roll_rad);

/* d = sqrt(x^2 + y^2) and psi = atan2(y, x) of a body-frame point (MAP-04). */
float geo_horiz_dist(geo_vec3_t p_b);
float geo_azimuth_deg(geo_vec3_t p_b);

/* Point at range r along the centre ray of zone (i, j), sensor frame. */
geo_vec3_t geo_zone_point_sensor(const geo_mount_t *m, uint8_t i, uint8_t j, float r_m);

/* Horizontal distance and azimuth in the body frame of a target at range r in zone (i, j). */
void geo_zone_target(const geo_mount_t *m, uint8_t i, uint8_t j, float r_m, float *d_m, float *az_deg);

/* Angular extent of zone (i, j) by ray directions (range -> infinity); see geo_zone_extent_t. */
void geo_zone_extent(const geo_mount_t *m, uint8_t i, uint8_t j, geo_zone_extent_t *out);

/* MAP-05a: extent in azimuths seen from the body centre at range r_m, with t_m (parallax). */
void geo_zone_extent_at_range(const geo_mount_t *m, uint8_t i, uint8_t j, float r_m, geo_zone_extent_t *out);

/* MAP-14a: h_p = h_body - z_L(p_B), p_B with t_m; all heights refer to the body centre. */
float geo_point_height(const geo_mount_t *m, geo_vec3_t p_s, float pitch_rad, float roll_rad, float h_body_m);

/* MAP-14a: body-centre height from the optics height of a sensor (on the ground:
 * TOF_HEIGHT_GROUND_M with MOUNT_XYZ_M): h_body = h_opt + z_L(t_m). */
float geo_body_height_from_sensor(const geo_mount_t *m, float h_opt_m, float pitch_rad, float roll_rad);

/* MAP-14a: body-centre height from a downward rangefinder at t_rf (RF_XYZ_M), flat ground:
 * h_body = range * cos(pitch) * cos(roll) + z_L(t_rf). */
float geo_body_height_from_rf(float range_m, geo_vec3_t t_rf, float pitch_rad, float roll_rad);

#endif /* CORE_GEOMETRY_H */
