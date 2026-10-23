#ifndef CORE_GROUND_H
#define CORE_GROUND_H

#include <stdbool.h>
#include <stdint.h>

#include "core/geometry.h"
#include "core/phase.h"

/* "Ray - ground" check (MAP-14, MAP-14a, MAP-18) and the body height source by phase (MAP-15).
 * Status (MAP-19): an approximate model - a zone is a cone, not a ray; telling the ground from low
 * obstacles of 5-20 cm is NOT confirmed until T-25 and T-26. GROUND_POLICY = DROP stays forbidden
 * until then (MAP-07, MAP-07a). */

/* Context of one sensor frame. Without valid orientation or height no target is flagged. */
typedef struct {
    bool valid;        /* ATTITUDE at capture and h_body are valid: classification on */
    float pitch_rad;   /* ATTITUDE interpolated at the frame time */
    float roll_rad;
    float h_body_m;    /* body-centre height (MAP-14a) */
    float tol_m;       /* GROUND_TOL_M */
    float cal_tol_deg;    /* CAL_TOL_DEG: azimuth tolerance */
    float cal_tol_el_deg; /* CAL_TOL_EL_DEG: elevation tolerance (MAP-14b) */
} ground_ctx_t;

/* MAP-14, MAP-14a, MAP-14b: lowest point height h_p = h_body - z_L(p_B) over every direction inside
 * the angular extent of zone (i, j) widened by CAL_TOL_DEG in azimuth and CAL_TOL_EL_DEG in elevation,
 * at range r_m (p_B with t_m). Exact maximum of z_L over the alpha/beta box, not a sampling. A zone
 * whose upper edge reaches the horizon is ground-compatible at any range beyond the touch point: a
 * limitation of the model (MAP-19), not a defect. */
float ground_min_height(const geo_mount_t *m, uint8_t i, uint8_t j, float r_m, const ground_ctx_t *g);

/* MAP-14: a target at range r_m is compatible with the ground if some direction of the zone extent
 * (+tolerances, MAP-14b) gives h_p <= GROUND_TOL_M. False when g is NULL or not valid. */
bool ground_target_compatible(const geo_mount_t *m, uint8_t i, uint8_t j, float r_m, const ground_ctx_t *g);

typedef enum {
    GROUND_H_NONE = 0,     /* no allowed source: classification off */
    GROUND_H_RANGEFINDER,  /* FC rangefinder with RF_XYZ_M */
    GROUND_H_PARAM         /* TOF_HEIGHT_GROUND_M with MOUNT_XYZ_M */
} ground_h_src_t;

/* MAP-15, MAP-15b: GROUND, ARMED_GROUND, GROUND_RECOVER - parameter; TAKEOFF, FLIGHT, LANDING,
 * PHASE_UNKNOWN - rangefinder if valid (within 500 ms), else none: classification off, every target real.
 * TOF_HEIGHT_GROUND_M is never used in the air. */
ground_h_src_t ground_height_source(phase_t ph, bool rf_valid);

typedef struct {
    bool rf_valid;
    float rf_range_m;   /* DISTANCE_SENSOR orientation 25 */
    geo_vec3_t rf_xyz;  /* RF_XYZ_M */
    float h_opt_ground_m; /* TOF_HEIGHT_GROUND_M */
} ground_height_in_t;

/* MAP-14a, MAP-15: body-centre height by phase; the parameter source is corrected with the mount of
 * `m`. Returns the source used; *h_m is left unchanged for GROUND_H_NONE. */
ground_h_src_t ground_body_height(phase_t ph, const ground_height_in_t *in, const geo_mount_t *m,
                                  float pitch_rad, float roll_rad, float *h_m);

#endif /* CORE_GROUND_H */
