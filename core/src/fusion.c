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

/* REQ: MAP-05, MAP-04 (ROW_MASK), MAP-09, MAP-09a, SNS-12a */
void fusion_init(fusion_t *f, const params_t *p)
{
    memset(f, 0, sizeof *f);
    f->res = (uint8_t)params_get_u32(p, PARAM_SENSOR_RES);
    quality_cfg_from_params(&f->qcfg, p);
    cell_cfg_from_params(&f->ccfg, p);
    f->staggered = params_get_u32(p, PARAM_SCHEDULE) == PARAM_SCHEDULE_STAGGERED;
    f->orient_confirmed = params_get_u32(p, PARAM_ORIENT_CONFIRMED) != 0u;
    f->min_cm = (uint16_t)params_get_u32(p, PARAM_MIN_DIST_CM);
    f->max_cm = (uint16_t)params_get_u32(p, PARAM_MAX_DIST_CM);
    f->ambient_clear_max = params_get_u32(p, PARAM_AMBIENT_CLEAR_MAX);
    f->ambient_degraded = params_get_u32(p, PARAM_AMBIENT_DEGRADED);
    const uint32_t win_ms = params_get_u32(p, PARAM_FUSION_WINDOW_MS);
    /* 0 = start value of section 15: 1.5 frame periods. */
    f->window_us = (win_ms > 0u) ? win_ms * 1000u : 1500000u / params_get_u32(p, PARAM_SENSOR_FREQ_HZ);
    f->max_age_us = params_get_u32(p, PARAM_VALID_DATA_MAX_AGE_MS) * 1000u;
    f->cal_tol_deg = params_get_f32(p, PARAM_CAL_TOL_DEG);

    const uint32_t row_mask = params_get_u32(p, PARAM_ROW_MASK);
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
            f->zone_i[s][z] = i;
            f->zone_j[s][z] = j;
            f->ray_b[s][z] = geo_mat_apply(
                f->mount[s].r_bs,
                geo_dir_sensor(geo_zone_alpha_deg(f->res, j), geo_zone_beta_deg(f->res, i)));
            geo_zone_extent(&f->mount[s], i, j, &f->extent[s][z]);
            if (((row_mask >> i) & 1u) == 0u) {
                continue; /* MAP-04: masked rows do not enter the map */
            }
            f->row_ok[s] |= (uint64_t)1u << z;
            const geo_zone_extent_t e = f->extent[s][z];
            for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
                if (sector_hit(e.az_min_deg, e.az_max_deg, f->cal_tol_deg, k)) {
                    f->sector_zones[k][s] |= (uint64_t)1u << z;
                }
            }
        }
    }
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
            f->in_fov[k] = f->in_fov[k] || (f->sector_zones[k][s] != 0u);
        }
        cell_init(&f->cell[k]);
        cell_init(&f->cell_real[k]);
    }
    f->state = f->orient_confirmed ? MAP_OK : MAP_DEGRADED;
}

/* Nearest target of a sector; on equal distance a real obstacle wins over a ground point. */
static void sec_take(fusion_sec_occ_t *o, uint8_t z, uint16_t d_cm, bool ground, bool angle_ok)
{
    if (!o->hit || d_cm < o->dist_cm || (d_cm == o->dist_cm && o->ground && !ground)) {
        o->hit = true;
        o->dist_cm = d_cm;
        o->zone = z;
        o->angle_ok = angle_ok;
        o->ground = ground;
    }
}

/* REQ: MAP-05a, MAP-18. A valid obstacle target at range r: its zone extent is recomputed in
 * azimuths from the body centre at that range (with t_m) and it is entered into every in-FoV sector
 * it hits; a ground-compatible target only into FUSION_VIEW_ALL. */
static void mark_target(const fusion_t *f, uint8_t s, uint8_t z, float r, uint16_t d_cm, bool ground,
                        fusion_frame_t *ff)
{
    geo_zone_extent_t e;
    geo_zone_extent_at_range(&f->mount[s], f->zone_i[s][z], f->zone_j[s][z], r, &e);
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        if (!f->in_fov[k] || !sector_hit(e.az_min_deg, e.az_max_deg, f->cal_tol_deg, k)) {
            continue;
        }
        const bool angle_ok = sector_hit(e.az_min_deg, e.az_max_deg, 0.0f, k);
        sec_take(&ff->sec_occ[k], z, d_cm, ground, angle_ok);
        if (!ground) {
            sec_take(&ff->sec_real[k], z, d_cm, false, angle_ok);
        }
    }
}

/* REQ: MAP-03, MAP-04, MAP-05a, MAP-13, MAP-13a, MAP-14, MAP-18 (zone candidate: OCC / CLR / INV
 * in both views, GROUND_SUSPECT flag) */
static void classify_zone(const fusion_t *f, uint8_t s, uint8_t z, const tof_zone_t *zone,
                          const ground_ctx_t *g, fusion_frame_t *ff, quality_zone_t *q)
{
    quality_zone_eval(&f->qcfg, zone, q);
    fusion_zone_t *out = &ff->zone[z];
    const bool in_map = ((f->row_ok[s] >> z) & 1u) != 0u;

    bool found = false;
    bool found_real = false;
    bool too_close = false;
    uint16_t best = 0u;
    for (uint8_t k = 0; k < q->n_targets; k++) {
        if (((q->valid_mask >> k) & 1u) == 0u) {
            continue;
        }
        if (zone->target[k].distance_mm <= 0) {
            too_close = true;
            continue;
        }
        /* MAP-05a: the point is built in the body frame with MOUNT_XYZ_M. */
        const float r = (float)zone->target[k].distance_mm / 1000.0f;
        const geo_vec3_t u = f->ray_b[s][z];
        const geo_vec3_t t = f->mount[s].t_m;
        const geo_vec3_t p_b = {u.x * r + t.x, u.y * r + t.y, u.z * r + t.z};
        const float d = geo_horiz_dist(p_b) * 100.0f;
        const uint16_t d_cm = (d >= 65535.0f) ? 65535u : (uint16_t)d; /* floor: nearer */
        if (d_cm < f->min_cm) {
            too_close = true;
        } else if (d_cm <= f->max_cm) {
            if (!found || d_cm < best) {
                found = true;
                best = d_cm;
            }
            /* MAP-14, MAP-18: each target is checked against the ground on its own. */
            const bool ground =
                ground_target_compatible(&f->mount[s], f->zone_i[s][z], f->zone_j[s][z], r, g);
            found_real = found_real || !ground;
            if (in_map) {
                mark_target(f, s, z, r, d_cm, ground, ff);
            }
        }
    }
    out->dist_cm = best;
    const bool all_valid_beyond = (q->n_valid == q->n_targets) && !too_close;
    if (found) {
        out->cls = ZONE_OCC;
    } else if (f->ccfg.clear_enabled) {
        /* MAP-13a: CLEAR_MEASURED on - CLR only with ambient <= AMBIENT_CLEAR_MAX. */
        out->cls = (all_valid_beyond && zone->ambient_per_spad <= f->ambient_clear_max) ? ZONE_CLR
                                                                                       : ZONE_INV;
    } else {
        /* MAP-13a: CLEAR_MEASURED off - CLR only from a valid target beyond max_distance;
         * a zone without a target is INV. */
        out->cls = (q->has_target && all_valid_beyond) ? ZONE_CLR : ZONE_INV;
    }
    /* MAP-18: GROUND_SUSPECT only if every valid target in range is ground-compatible; any other
     * target makes the zone OCC (nearest such target). Ground does not confirm absence: INV. */
    out->ground = found && !found_real;
    out->cls_real = found_real ? (uint8_t)ZONE_OCC : (out->ground ? (uint8_t)ZONE_INV : out->cls);
}

/* Median of ambient_per_spad over the zones of a frame (MAP-10a). */
static uint32_t ambient_median(const tof_frame_t *frame, uint8_t nz)
{
    uint32_t v[TOF_MAX_ZONES];
    for (uint8_t z = 0; z < nz; z++) {
        /* insertion sort, at most 64 values */
        const uint32_t a = frame->zone[z].ambient_per_spad;
        uint8_t k = z;
        while (k > 0u && v[k - 1u] > a) {
            v[k] = v[k - 1u];
            k--;
        }
        v[k] = a;
    }
    return v[nz / 2u];
}

/* REQ: SNS-09, MAP-03, MAP-10a (valid fraction, ambient median), MAP-14, MAP-18 */
bool fusion_push_frame(fusion_t *f, const tof_frame_t *frame, const ground_ctx_t *g)
{
    if (frame->sensor_id >= TOF_NUM_SENSORS || frame->res != f->res) {
        return false;
    }
    const uint8_t s = frame->sensor_id;
    fusion_frame_t *ff = &f->frame[s];
    const uint8_t nz = (uint8_t)(f->res * f->res);
    memset(ff->sec_occ, 0, sizeof ff->sec_occ);
    memset(ff->sec_real, 0, sizeof ff->sec_real);
    uint8_t n_with = 0u;
    uint8_t n_valid = 0u;
    uint8_t n_ground = 0u;
    for (uint8_t z = 0; z < nz; z++) {
        quality_zone_t q;
        classify_zone(f, s, z, &frame->zone[z], g, ff, &q);
        n_with = (uint8_t)(n_with + (q.has_target ? 1u : 0u));
        n_valid = (uint8_t)(n_valid + (q.zone_valid ? 1u : 0u));
        n_ground = (uint8_t)(n_ground + (ff->zone[z].ground ? 1u : 0u));
    }
    ff->n_ground_zones = n_ground;
    ff->has_data = true;
    ff->fresh = true;
    ff->t_us = frame->t_ready_us;
    ff->frame_no = frame->frame_no;
    /* MAP-10a: zones without a target are not counted. */
    ff->valid_frac = (n_with > 0u) ? (float)n_valid / (float)n_with : 1.0f;
    ff->ambient_median = ambient_median(frame, nz);
    ff->ambient_high = f->ambient_degraded > 0u && ff->ambient_median > f->ambient_degraded;
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

/* REQ: MAP-11, SNS-12a */
map_state_t fusion_map_state_cfg(const fusion_t *f, const sensor_health_t health[TOF_NUM_SENSORS])
{
    const map_state_t st = fusion_map_state(health);
    return (st == MAP_OK && !f->orient_confirmed) ? MAP_DEGRADED : st;
}

/* One sector of one view: candidate over all zones whose extent hits it (5.3 overlap rules), then
 * MAP-13. */
/* REQ: MAP-05, MAP-05a, MAP-09, MAP-10, MAP-13, MAP-17 (views) */
static void step_sector(fusion_t *f, fusion_view_t view, uint8_t k, const bool used[TOF_NUM_SENSORS],
                        uint32_t now_us)
{
    const bool real = view == FUSION_VIEW_REAL;
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
        const fusion_frame_t *ff = &f->frame[s];
        const fusion_sec_occ_t *so = real ? &ff->sec_real[k] : &ff->sec_occ[k];
        const bool occ_here = used[s] && so->hit;
        if (mask == 0u && !occ_here) {
            continue;
        }
        if (!used[s]) {
            all_clr = false; /* no data, or FAILED sensor (MAP-10): INV */
            continue;
        }
        const uint32_t age = age_us(now_us, ff->t_us);
        if (!have_t || age > oldest_age) {
            have_t = true;
            oldest_age = age;
            t_oldest = ff->t_us;
        }
        /* CLR needs every zone of the sector to agree; an OCC zone whose target lies in another
         * sector (parallax) does not confirm absence here either. */
        for (uint8_t z = 0; z < nz; z++) {
            const uint8_t cls = real ? ff->zone[z].cls_real : ff->zone[z].cls;
            if (((mask >> z) & 1u) != 0u && cls != ZONE_CLR) {
                all_clr = false;
            }
        }
        /* Nearest valid target wins; INV of another zone does not cancel it. */
        if (occ_here && (!found || so->dist_cm < in.dist_cm ||
                         (so->dist_cm == in.dist_cm && in.ground_suspect && !so->ground))) {
            found = true;
            in.dist_cm = so->dist_cm;
            in.src_sensor = s;
            in.src_zone = so->zone;
            in.angle_ok = so->angle_ok;
            in.ground_suspect = so->ground;
            t_best = ff->t_us;
        }
    }

    cell_t *cell = real ? &f->cell_real[k] : &f->cell[k];
    uint8_t evt;
    if (found) {
        in.kind = CELL_CAND_OCC;
        evt = cell_fsm_step(cell, &f->ccfg, &in, t_best); /* measurement time */
    } else {
        in.kind = all_clr ? CELL_CAND_CLR : CELL_CAND_INV;
        evt = cell_fsm_step(cell, &f->ccfg, &in, now_us);
    }
    if ((evt & CELL_EVT_HOLD_EXPIRED) != 0u) {
        f->hold_expired[view][k] = true; /* MAP-13: logged by the scheduler with sector and time */
        f->hold_expired_total += real ? 0u : 1u;
    }
    if (have_t && !real) {
        f->cell_t_us[k] = t_oldest; /* MAP-09: time of the oldest contribution */
        f->cell_has_time[k] = true;
    }
}

static void step_all(fusion_t *f, const sensor_health_t health[TOF_NUM_SENSORS],
                     const bool used[TOF_NUM_SENSORS], uint32_t now_us)
{
    memset(f->hold_expired, 0, sizeof f->hold_expired);
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        if (f->in_fov[k]) {
            step_sector(f, FUSION_VIEW_ALL, k, used, now_us);
            step_sector(f, FUSION_VIEW_REAL, k, used, now_us);
        }
    }
    f->state = fusion_map_state_cfg(f, health);
    f->t_map_us = now_us;
    f->map_seq++;
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
    bool have_t1 = false;
    uint32_t t1_age = 0u;
    for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
        used[s] = false;
        if (!f->frame[s].fresh) {
            continue;
        }
        const uint32_t age = age_us(now_us, f->frame[s].t_us);
        if (age - newest_age > f->window_us) {
            f->frames_late_total++; /* older than the newest frame by more than the window */
        } else {
            used[s] = health[s] != SENSOR_FAILED;
            if (!have_t1 || age > t1_age) {
                /* t1 of a SIMULTANEOUS map: the oldest frame used */
                have_t1 = true;
                t1_age = age;
                f->t1_us = f->frame[s].t_us;
                f->t1_sensor = s;
            }
        }
        f->frame[s].fresh = false;
    }
    step_all(f, health, used, now_us);
}

/* REQ: MAP-09a */
void fusion_run_staggered(fusion_t *f, const sensor_health_t health[TOF_NUM_SENSORS], uint8_t trigger,
                          uint32_t now_us)
{
    const uint32_t t1 = f->frame[trigger].t_us;
    bool used[TOF_NUM_SENSORS];
    for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
        const fusion_frame_t *ff = &f->frame[s];
        used[s] = ff->has_data && health[s] != SENSOR_FAILED && age_us(t1, ff->t_us) <= f->max_age_us;
        f->frame[s].fresh = false;
    }
    f->t1_us = t1;
    f->t1_sensor = trigger;
    step_all(f, health, used, now_us);
}

/* REQ: MAP-09 (STALE), 5.2, MAP-17 (view) */
export_state_t fusion_export_state_view(const fusion_t *f, fusion_view_t view, uint8_t sector,
                                        uint32_t now_us)
{
    if (!f->in_fov[sector]) {
        return EXPORT_OUT_OF_FOV;
    }
    if (!f->cell_has_time[sector] || age_us(now_us, f->cell_t_us[sector]) > f->max_age_us) {
        return EXPORT_STALE;
    }
    const cell_t *c = (view == FUSION_VIEW_REAL) ? &f->cell_real[sector] : &f->cell[sector];
    switch (c->state) {
    case CELL_OCCUPIED:
        return EXPORT_OCCUPIED;
    case CELL_CLEAR_MEASURED:
        return EXPORT_CLEAR_MEASURED;
    default:
        return EXPORT_UNKNOWN;
    }
}

/* REQ: 5.2 */
export_state_t fusion_export_state(const fusion_t *f, uint8_t sector, uint32_t now_us)
{
    return fusion_export_state_view(f, FUSION_VIEW_ALL, sector, now_us);
}

/* REQ: MAP-05, MAP-17, 5.2 (OCCUPIED -> cm, CLEAR_MEASURED -> max + 1, UNKNOWN / STALE / out of
 * FoV -> 65535) */
void fusion_export_view(const fusion_t *f, fusion_view_t view, uint32_t now_us,
                        uint16_t distances[TOF_NUM_SECTORS])
{
    const cell_t *cells = (view == FUSION_VIEW_REAL) ? f->cell_real : f->cell;
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        switch (fusion_export_state_view(f, view, k, now_us)) {
        case EXPORT_OCCUPIED:
            distances[k] = cells[k].dist_cm;
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

/* REQ: 5.2 */
void fusion_export(const fusion_t *f, uint32_t now_us, uint16_t distances[TOF_NUM_SECTORS])
{
    fusion_export_view(f, FUSION_VIEW_ALL, now_us, distances);
}

/* REQ: MAP-16 (GROUND_RECOVER: reset of cell automata and holds) */
void fusion_reset_cells(fusion_t *f)
{
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        cell_init(&f->cell[k]);
        cell_init(&f->cell_real[k]);
    }
}
