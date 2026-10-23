#include "core/ground.h"

#include <math.h>
#include <stddef.h>

/* Maximum of a*cos(x) + b*sin(x) for x in [lo, hi] (radians, hi - lo < pi): at an end, or at
 * atan2(b, a) if that lies inside. */
static float max_cos_sin(float a, float b, float lo, float hi)
{
    float best = a * cosf(lo) + b * sinf(lo);
    const float v_hi = a * cosf(hi) + b * sinf(hi);
    best = (v_hi > best) ? v_hi : best;
    const float x = atan2f(b, a);
    if (x > lo && x < hi) {
        const float v = a * cosf(x) + b * sinf(x);
        best = (v > best) ? v : best;
    }
    return best;
}

/* REQ: MAP-14, MAP-14a, MAP-14b */
float ground_min_height(const geo_mount_t *m, uint8_t i, uint8_t j, float r_m, const ground_ctx_t *g)
{
    /* z_L of a direction (alpha, beta) of the sensor: c . [cos b cos a, cos b sin a, -sin b], with c the
     * z row of Ry(pitch) Rx(roll) R_bs. For |b| < 90 deg the alpha part is maximised independently. */
    const geo_mat3_t lv = geo_mat_mul(geo_rot_y(g->pitch_rad), geo_rot_x(g->roll_rad));
    const geo_mat3_t ls = geo_mat_mul(lv, m->r_bs);
    const float c0 = ls.m[2][0];
    const float c1 = ls.m[2][1];
    const float c2 = ls.m[2][2];
    /* MAP-14b: CAL_TOL_DEG in azimuth only, CAL_TOL_EL_DEG in elevation. */
    const float half_a = 0.5f * geo_zone_width_deg(m->n) + g->cal_tol_deg;
    const float half_b = 0.5f * geo_zone_width_deg(m->n) + g->cal_tol_el_deg;
    const float a_c = geo_zone_alpha_deg(m->n, j);
    const float b_c = geo_zone_beta_deg(m->n, i);
    const float k = max_cos_sin(c0, c1, geo_deg_to_rad(a_c - half_a), geo_deg_to_rad(a_c + half_a));
    const float d = max_cos_sin(k, -c2, geo_deg_to_rad(b_c - half_b), geo_deg_to_rad(b_c + half_b));
    const float z_t = geo_mat_apply(lv, m->t_m).z;
    return g->h_body_m - (r_m * d + z_t);
}

/* REQ: MAP-14, MAP-18 (each target separately) */
bool ground_target_compatible(const geo_mount_t *m, uint8_t i, uint8_t j, float r_m, const ground_ctx_t *g)
{
    if (g == NULL || !g->valid) {
        return false; /* MAP-14: without valid orientation the flag is not set */
    }
    return ground_min_height(m, i, j, r_m, g) <= g->tol_m;
}

/* REQ: MAP-15, MAP-15b */
ground_h_src_t ground_height_source(phase_t ph, bool rf_valid)
{
    switch (ph) {
    case PHASE_GROUND:
    case PHASE_ARMED_GROUND:
    case PHASE_GROUND_RECOVER:
        return GROUND_H_PARAM;
    default: /* TAKEOFF, FLIGHT, LANDING, PHASE_UNKNOWN: never the ground parameter in the air */
        return rf_valid ? GROUND_H_RANGEFINDER : GROUND_H_NONE;
    }
}

/* REQ: MAP-14a, MAP-15 */
ground_h_src_t ground_body_height(phase_t ph, const ground_height_in_t *in, const geo_mount_t *m,
                                  float pitch_rad, float roll_rad, float *h_m)
{
    const ground_h_src_t src = ground_height_source(ph, in->rf_valid);
    if (src == GROUND_H_RANGEFINDER) {
        *h_m = geo_body_height_from_rf(in->rf_range_m, in->rf_xyz, pitch_rad, roll_rad);
    } else if (src == GROUND_H_PARAM) {
        *h_m = geo_body_height_from_sensor(m, in->h_opt_ground_m, pitch_rad, roll_rad);
    } else {
        /* no source: *h_m unchanged */
    }
    return src;
}
