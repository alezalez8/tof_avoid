#ifndef CORE_FUSION_H
#define CORE_FUSION_H

#include <stdbool.h>
#include <stdint.h>

#include "core/cell_fsm.h"
#include "core/config.h"
#include "core/frame.h"
#include "core/geometry.h"
#include "core/params.h"
#include "core/quality.h"
#include "core/sensor_health.h"

/* Obstacle map: 72 x 5 deg grid (MAP-05, MAP-05a) built from the frames of three sensors
 * (MAP-09, MAP-09a), cell hold/release by MAP-13, map state (MAP-11, SNS-12a), export by 5.2. */

#define FUSION_DIST_UNKNOWN 65535u /* "no data" for ArduPilot, never "free" */

typedef enum {
    ZONE_INV = 0,
    ZONE_OCC,
    ZONE_CLR
} zone_class_t;

typedef struct {
    uint8_t cls;      /* zone_class_t */
    uint16_t dist_cm; /* ZONE_OCC: horizontal distance of the nearest valid target in range */
} fusion_zone_t;

/* Nearest obstacle of one frame in one sector, from the per-target extent at range (MAP-05a). */
typedef struct {
    bool hit;
    bool angle_ok; /* the target's extent hits the sector without CAL_TOL_DEG */
    uint8_t zone;  /* raw zone index */
    uint16_t dist_cm;
} fusion_sec_occ_t;

/* One sensor frame after zone classification, indexed by raw ULD zone. */
typedef struct {
    bool has_data;    /* a frame was received (STAGGERED reuses the last one) */
    bool fresh;       /* not yet used by a SIMULTANEOUS fusion step */
    uint32_t t_us;    /* t_ready_us of the frame */
    uint32_t frame_no;
    float valid_frac; /* MAP-10a: valid / zones with a target; 1 when no zone has a target */
    uint32_t ambient_median;
    bool ambient_high; /* median ambient > AMBIENT_DEGRADED (MAP-10a), never when the limit is 0 */
    fusion_zone_t zone[TOF_MAX_ZONES];
    fusion_sec_occ_t sec_occ[TOF_NUM_SECTORS];
} fusion_frame_t;

typedef enum {
    MAP_OK = 0,
    MAP_DEGRADED, /* at least one sensor not OK, or orientation not confirmed (SNS-12a) */
    MAP_LOST      /* no sensor OK */
} map_state_t;

/* Export classification of one sector (5.2). */
typedef enum {
    EXPORT_OUT_OF_FOV = 0,
    EXPORT_UNKNOWN,
    EXPORT_STALE,
    EXPORT_OCCUPIED,
    EXPORT_CLEAR_MEASURED
} export_state_t;

typedef struct {
    /* Configuration, fixed at init. */
    geo_mount_t mount[TOF_NUM_SENSORS];
    geo_vec3_t ray_b[TOF_NUM_SENSORS][TOF_MAX_ZONES]; /* unit centre ray in the body frame, raw index */
    geo_zone_extent_t extent[TOF_NUM_SENSORS][TOF_MAX_ZONES]; /* MAP-02: stored extent, raw index */
    uint8_t zone_i[TOF_NUM_SENSORS][TOF_MAX_ZONES]; /* (i, j) after ORIENT_TRANSFORM, raw index */
    uint8_t zone_j[TOF_NUM_SENSORS][TOF_MAX_ZONES];
    uint64_t row_ok[TOF_NUM_SENSORS]; /* raw zones not excluded by ROW_MASK */
    uint64_t sector_zones[TOF_NUM_SECTORS][TOF_NUM_SENSORS]; /* raw-zone masks, direction extent + CAL_TOL */
    bool in_fov[TOF_NUM_SECTORS];
    quality_cfg_t qcfg;
    cell_cfg_t ccfg;
    uint8_t res;
    bool staggered;        /* SCHEDULE = STAGGERED (MAP-09a) */
    bool orient_confirmed; /* ORIENT_CONFIRMED (SNS-12a) */
    float cal_tol_deg;
    uint16_t min_cm;
    uint16_t max_cm;
    uint32_t ambient_clear_max;
    uint32_t ambient_degraded;
    uint32_t window_us;
    uint32_t max_age_us;

    /* State. */
    fusion_frame_t frame[TOF_NUM_SENSORS];
    cell_t cell[TOF_NUM_SECTORS];
    uint32_t cell_t_us[TOF_NUM_SECTORS]; /* data time: oldest contribution of the last step */
    bool cell_has_time[TOF_NUM_SECTORS];
    map_state_t state;
    uint32_t map_seq;            /* fused maps produced */
    uint32_t t_map_us;           /* time of the last fused map */
    uint32_t t1_us;              /* MAP-12 t1 of the last map (MAP-09a) */
    uint32_t hold_expired_total; /* MAP-13 log events */
    uint32_t frames_late_total;  /* frames dropped as older than the fusion window */
} fusion_t;

void fusion_init(fusion_t *f, const params_t *p);

/* Classifies and stores a frame. False if its sensor id or resolution does not match. */
bool fusion_push_frame(fusion_t *f, const tof_frame_t *frame);

/* SIMULTANEOUS: true when every non-FAILED sensor has a fresh frame, or the oldest fresh frame has
 * waited for the fusion window. */
bool fusion_ready(const fusion_t *f, const sensor_health_t health[TOF_NUM_SENSORS], uint32_t now_us);

/* SIMULTANEOUS: one fusion step over the fresh frames within the window. */
void fusion_run(fusion_t *f, const sensor_health_t health[TOF_NUM_SENSORS], uint32_t now_us);

/* STAGGERED (MAP-09a): one step on the new frame of `trigger` with the last frames of the others
 * not older than VALID_DATA_MAX_AGE_MS; t1 is the trigger frame time. */
void fusion_run_staggered(fusion_t *f, const sensor_health_t health[TOF_NUM_SENSORS], uint8_t trigger,
                          uint32_t now_us);

/* Map state from sensor health only (MAP-11). */
map_state_t fusion_map_state(const sensor_health_t health[TOF_NUM_SENSORS]);

/* MAP-11 limited by SNS-12a: never OK while ORIENT_CONFIRMED = 0. */
map_state_t fusion_map_state_cfg(const fusion_t *f, const sensor_health_t health[TOF_NUM_SENSORS]);

export_state_t fusion_export_state(const fusion_t *f, uint8_t sector, uint32_t now_us);

/* distances[72] in cm by 5.2: OCCUPIED -> distance, CLEAR_MEASURED -> max + 1, else 65535. */
void fusion_export(const fusion_t *f, uint32_t now_us, uint16_t distances[TOF_NUM_SECTORS]);

#endif /* CORE_FUSION_H */
