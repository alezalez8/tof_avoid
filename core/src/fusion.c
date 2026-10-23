#include "core/fusion.h"

#include <math.h>
#include <string.h>

static const param_id_t k_yaw[TOF_NUM_SENSORS] = {PARAM_MOUNT_YAW_DEG_L, PARAM_MOUNT_YAW_DEG_C,
                                                  PARAM_MOUNT_YAW_DEG_R};
static const param_id_t k_pitch[TOF_NUM_SENSORS] = {PARAM_MOUNT_PITCH_DEG_L, PARAM_MOUNT_PITCH_DEG_C,
                                                    PARAM_MOUNT_PITCH_DEG_R};
static const param_id_t k_roll[TOF_NUM_SENSORS] = {PARAM_MOUNT_ROLL_DEG_L, PARAM_MOUNT_ROLL_DEG_C,
                                                   PARAM_MOUNT_ROLL_DEG_R};
static const param_id_t k_x[TOF_NUM_SENSORS] = {PARAM_MOUNT_X_M_L, PARAM_MOUNT_X_M_C,
                                                PARAM_MOUNT_X_M_R};
static const param_id_t k_y[TOF_NUM_SENSORS] = {PARAM_MOUNT_Y_M_L, PARAM_MOUNT_Y_M_C,
                                                PARAM_MOUNT_Y_M_R};
static const param_id_t k_z[TOF_NUM_SENSORS] = {PARAM_MOUNT_Z_M_L, PARAM_MOUNT_Z_M_C,
                                                PARAM_MOUNT_Z_M_R};
static const param_id_t k_orient[TOF_NUM_SENSORS] = {
    PARAM_ORIENT_TRANSFORM_L, PARAM_ORIENT_TRANSFORM_C, PARAM_ORIENT_TRANSFORM_R};

/* REQ: MAP-09. Wrap-safe age of a time stamp; a stamp slightly after now_us (frame stamped by the
 * ISR after the loop read the clock) counts as age 0 instead of wrapping to ~71 min. */
static uint32_t age_us(uint32_t now_us, uint32_t t_us)
{
    const uint32_t d = now_us - t_us;
    return (d > 0x80000000u) ? 0u : d;
}

/* Sector k covers [5k - 2.5, 5k + 2.5) deg; the zone covers [lo - tol, hi + tol]. Touching edges
 * do not count as an intersection. */
/* REQ: MAP-05 */
static bool sector_hit(float lo, float hi, float tol, uint8_t k)
{
    const float mid = 0.5f * (lo + hi);
    const float half = 0.5f * (hi - lo) + tol;
    const float delta = geo_wrap_deg_180((float)k * TOF_SECTOR_DEG - mid);
    return fabsf(delta) < 0.5f * TOF_SECTOR_DEG + half;
}

/* REQ: MAP-05, MAP-04 (ROW_MASK), MAP-09 (window) */
void fusion_init(fusion_t *f, const params_t *p)
{
    memset(f, 0, sizeof *f);
    f->res = (uint8_t)params_get_u32(p, PARAM_SENSOR_RES);
    quality_cfg_from_params(&f->qcfg, p);
    cell_cfg_from_params(&f->ccfg, p);
    f->min_cm = (uint16_t)params_get_u32(p, PARAM_MIN_DIST_CM);
    f->max_cm = (uint16_t)params_get_u32(p, PARAM_MAX_DIST_CM);
    f->ambient_clear_max = params_get_u32(p, PARAM_AMBIENT_CLEAR_MAX);
    const uint32_t win_ms = params_get_u32(p, PARAM_FUSION_WINDOW_MS);
    /* 0 = start value of section 15: 1.5 frame periods (DEV-08). */
    f->window_us = (win_ms > 0u) ? win_ms * 1000u : 1500000u / params_get_u32(p, PARAM_SENSOR_FREQ_HZ);
    f->max_age_us = params_get_u32(p, PARAM_VALID_DATA_MAX_AGE_MS) * 1000u;

    const uint32_t row_mask = params_get_u32(p, PARAM_ROW_MASK);
    const float tol = params_get_f32(p, PARAM_CAL_TOL_DEG);
    const uint8_t nz = (uint8_t)(f->res * f->res);
    for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
        const geo_vec3_t t = {params_get_f32(p, k_x[s]), params_get_f32(p, k_y[s]),
                              params_get_f32(p, k_z[s])};
        geo_mount_init(&f->mount[s], params_get_f32(p, k_yaw[s]), params_get_f32(p, k_pitch[s]),
                       params_get_f32(p, k_roll[s]), t, f->res,
                       (uint8_t)params_get_u32(p, k_orient[s]));
        for (uint8_t z = 0; z < nz; z++) {
            uint8_t i = 0u;
            uint8_t j = 0u;
            geo_zone_from_raw(&f->mount[s], z, &i, &j);
            f->ray_b[s][z] = geo_mat_apply(
                f->mount[s].r_bs,
                geo_dir_sensor(geo_zone_alpha_deg(f->res, j), geo_zone_beta_deg(f->res, i)));
            geo_zone_extent(&f->mount[s], i, j, &f->extent[s][z]);
            if (((row_mask >> i) & 1u) == 0u) {
                continue; /* MAP-04: masked rows do not enter the map */
            }
            const geo_zone_extent_t e = f->extent[s][z];
            for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
                if (sector_hit(e.az_min_deg, e.az_max_deg, tol, k)) {
                    f->sector_zones[k][s] |= (uint64_t)1u << z;
                }
                if (sector_hit(e.az_min_deg, e.az_max_deg, 0.0f, k)) {
                    f->sector_core[k][s] |= (uint64_t)1u << z;
                }
            }
        }
    }
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
            f->in_fov[k] = f->in_fov[k] || (f->sector_zones[k][s] != 0u);
        }
        cell_init(&f->cell[k]);
    }
    f->state = MAP_OK;
}

/* REQ: MAP-03, MAP-04, MAP-13 (zone candidate: OCC / CLR / INV) */
static void classify_zone(const fusion_t *f, uint8_t s, uint8_t z, const tof_zone_t *zone,
                          fusion_zone_t *out, bool *zone_valid)
{
    quality_zone_t q;
    quality_zone_eval(&f->qcfg, zone, &q);
    *zone_valid = q.zone_valid;

    bool found = false;
    bool too_close = false;
    uint16_t best = 0u;
    for (uint8_t k = 0; k < q.n_targets; k++) {
        if (((q.valid_mask >> k) & 1u) == 0u) {
            continue;
        }
        if (zone->target[k].distance_mm <= 0) {
            too_close = true;
            continue;
        }
        const float r = (float)zone->target[k].distance_mm / 1000.0f;
        const geo_vec3_t u = f->ray_b[s][z];
        const geo_vec3_t t = f->mount[s].t_m;
        const geo_vec3_t p_b = {u.x * r + t.x, u.y * r + t.y, u.z * r + t.z};
        const float d = geo_horiz_dist(p_b) * 100.0f;
        const uint16_t d_cm = (d >= 65535.0f) ? 65535u : (uint16_t)d; /* floor: nearer */
        if (d_cm < f->min_cm) {
            too_close = true;
        } else if (d_cm <= f->max_cm && (!found || d_cm < best)) {
            found = true;
            best = d_cm;
        }
    }
    out->dist_cm = best;
    if (found) {
        out->cls = ZONE_OCC;
    } else if ((!f->ccfg.clear_enabled || zone->ambient_per_spad <= f->ambient_clear_max) &&
               q.n_valid == q.n_targets && !too_close) {
        /* 5.3 CLR: every reported target valid and beyond max_distance. The ambient limit applies
         * only when CLEAR_MEASURED is enabled; when it is off, CLR releases to UNKNOWN (DEV-02). */
        out->cls = ZONE_CLR;
    } else {
        out->cls = ZONE_INV;
    }
}

/* REQ: SNS-09, MAP-03, MAP-10 (valid fraction) */
bool fusion_push_frame(fusion_t *f, const tof_frame_t *frame)
{
    if (frame->sensor_id >= TOF_NUM_SENSORS || frame->res != f->res) {
        return false;
    }
    fusion_frame_t *ff = &f->frame[frame->sensor_id];
    const uint8_t nz = (uint8_t)(f->res * f->res);
    uint8_t n_valid = 0u;
    for (uint8_t z = 0; z < nz; z++) {
        bool zv = false;
        classify_zone(f, frame->sensor_id, z, &frame->zone[z], &ff->zone[z], &zv);
        n_valid = (uint8_t)(n_valid + (zv ? 1u : 0u));
    }
    ff->fresh = true;
    ff->t_us = frame->t_ready_us;
    ff->frame_no = frame->frame_no;
    ff->valid_frac = (float)n_valid / (float)nz;
    return true;
}

/* REQ: MAP-09 */
bool fusion_ready(const fusion_t *f, const sensor_health_t health[TOF_NUM_SENSORS], uint32_t now_us)
{
    bool any = false;
    bool all = true;
    uint32_t oldest_age = 0u;
    for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
        if (f->frame[s].fresh) {
            const uint32_t age = age_us(now_us, f->frame[s].t_us);
            any = true;
            oldest_age = (age > oldest_age) ? age : oldest_age;
        } else if (health[s] != SENSOR_FAILED) {
            all = false;
        }
    }
    return any && (all || oldest_age >= f->window_us);
}

/* REQ: MAP-11 */
map_state_t fusion_map_state(const sensor_health_t health[TOF_NUM_SENSORS])
{
    uint8_t ok = 0u;
    for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
        ok = (uint8_t)(ok + (health[s] == SENSOR_OK ? 1u : 0u));
    }
    if (ok == TOF_NUM_SENSORS) {
        return MAP_OK;
    }
    return (ok == 0u) ? MAP_LOST : MAP_DEGRADED;
}

/* One sector: candidate over all zones whose extent hits it (5.3 overlap rules), then MAP-13. */
/* REQ: MAP-05, MAP-09, MAP-10, MAP-13 */
static void step_sector(fusion_t *f, uint8_t k, const bool used[TOF_NUM_SENSORS], uint32_t now_us)
{
    const uint8_t nz = (uint8_t)(f->res * f->res);
    cell_input_t in;
    memset(&in, 0, sizeof in);
    bool found = false;
    bool all_clr = true;
    bool have_t = false;
    uint32_t oldest_age = 0u;
    uint32_t t_oldest = 0u;
    uint32_t t_best = 0u;

    for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
        const uint64_t mask = f->sector_zones[k][s];
        if (mask == 0u) {
            continue;
        }
        if (!used[s]) {
            all_clr = false; /* no data, or FAILED sensor (MAP-10): INV */
            continue;
        }
        const fusion_frame_t *ff = &f->frame[s];
        const uint32_t age = age_us(now_us, ff->t_us);
        if (!have_t || age > oldest_age) {
            have_t = true;
            oldest_age = age;
            t_oldest = ff->t_us;
        }
        for (uint8_t z = 0; z < nz; z++) {
            if (((mask >> z) & 1u) == 0u) {
                continue;
            }
            const fusion_zone_t *zc = &ff->zone[z];
            if (zc->cls == ZONE_OCC) {
                /* Nearest valid target wins; INV of another zone does not cancel it. */
                if (!found || zc->dist_cm < in.dist_cm) {
                    found = true;
                    in.dist_cm = zc->dist_cm;
                    in.src_sensor = s;
                    in.src_zone = z;
                    in.angle_ok = ((f->sector_core[k][s] >> z) & 1u) != 0u;
                    t_best = ff->t_us;
                }
            } else if (zc->cls == ZONE_INV) {
                all_clr = false;
            }
        }
    }

    uint8_t evt;
    if (found) {
        in.kind = CELL_CAND_OCC;
        evt = cell_fsm_step(&f->cell[k], &f->ccfg, &in, t_best); /* measurement time */
    } else {
        in.kind = all_clr ? CELL_CAND_CLR : CELL_CAND_INV; /* CLR needs every zone to agree */
        evt = cell_fsm_step(&f->cell[k], &f->ccfg, &in, now_us);
    }
    if ((evt & CELL_EVT_HOLD_EXPIRED) != 0u) {
        f->hold_expired_total++;
    }
    if (have_t) {
        f->cell_t_us[k] = t_oldest; /* MAP-09: time of the oldest contribution */
        f->cell_has_time[k] = true;
    }
}

/* REQ: MAP-09, MAP-05, MAP-10, MAP-11 */
void fusion_run(fusion_t *f, const sensor_health_t health[TOF_NUM_SENSORS], uint32_t now_us)
{
    bool any = false;
    uint32_t newest_age = 0u;
    for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
        if (f->frame[s].fresh) {
            const uint32_t age = age_us(now_us, f->frame[s].t_us);
            newest_age = (!any || age < newest_age) ? age : newest_age;
            any = true;
        }
    }
    bool used[TOF_NUM_SENSORS];
    for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
        used[s] = false;
        if (!f->frame[s].fresh) {
            continue;
        }
        if (age_us(now_us, f->frame[s].t_us) - newest_age > f->window_us) {
            f->frames_late_total++; /* older than the newest frame by more than the window */
        } else {
            used[s] = health[s] != SENSOR_FAILED;
        }
        f->frame[s].fresh = false;
    }
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        if (f->in_fov[k]) {
            step_sector(f, k, used, now_us);
        }
    }
    f->state = fusion_map_state(health);
    f->t_map_us = now_us;
    f->map_seq++;
}

/* REQ: MAP-09 (STALE), 5.2 */
export_state_t fusion_export_state(const fusion_t *f, uint8_t sector, uint32_t now_us)
{
    if (!f->in_fov[sector]) {
        return EXPORT_OUT_OF_FOV;
    }
    if (!f->cell_has_time[sector] || age_us(now_us, f->cell_t_us[sector]) > f->max_age_us) {
        return EXPORT_STALE;
    }
    switch (f->cell[sector].state) {
    case CELL_OCCUPIED:
        return EXPORT_OCCUPIED;
    case CELL_CLEAR_MEASURED:
        return EXPORT_CLEAR_MEASURED;
    default:
        return EXPORT_UNKNOWN;
    }
}

/* REQ: MAP-05, 5.2 (OCCUPIED -> cm, CLEAR_MEASURED -> max + 1, UNKNOWN / STALE / out of FoV -> 65535) */
void fusion_export(const fusion_t *f, uint32_t now_us, uint16_t distances[TOF_NUM_SECTORS])
{
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        switch (fusion_export_state(f, k, now_us)) {
        case EXPORT_OCCUPIED:
            distances[k] = f->cell[k].dist_cm;
            break;
        case EXPORT_CLEAR_MEASURED:
            distances[k] = (uint16_t)(f->max_cm + 1u);
            break;
        default:
            distances[k] = FUSION_DIST_UNKNOWN;
            break;
        }
    }
}
