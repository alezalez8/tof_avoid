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

/* Obstacle map: 72 x 5 deg grid (MAP-05) built from the frames of three sensors fused within
 * FUSION_WINDOW_MS (MAP-09), cell hold/release by MAP-13, map state (MAP-11), export by 5.2. */

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

/* One sensor frame after zone classification, indexed by raw ULD zone. */
typedef struct {
    bool fresh;       /* not yet used by a fusion step */
    uint32_t t_us;    /* t_ready_us of the frame */
    uint32_t frame_no;
    float valid_frac; /* share of valid zones (MAP-10, DEV-03) */
    fusion_zone_t zone[TOF_MAX_ZONES];
} fusion_frame_t;

typedef enum {
    MAP_OK = 0,
    MAP_DEGRADED, /* at least one sensor not OK */
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
    uint64_t sector_zones[TOF_NUM_SECTORS][TOF_NUM_SENSORS]; /* raw-zone masks, extent + CAL_TOL */
    uint64_t sector_core[TOF_NUM_SECTORS][TOF_NUM_SENSORS];  /* same without CAL_TOL (angle_ok) */
    bool in_fov[TOF_NUM_SECTORS];
    quality_cfg_t qcfg;
    cell_cfg_t ccfg;
    uint8_t res;
    uint16_t min_cm;
    uint16_t max_cm;
    uint32_t ambient_clear_max;
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
    uint32_t hold_expired_total; /* MAP-13 log events */
    uint32_t frames_late_total;  /* frames dropped as older than the fusion window */
} fusion_t;

void fusion_init(fusion_t *f, const params_t *p);

/* Classifies and stores a frame. False if its sensor id or resolution does not match. */
bool fusion_push_frame(fusion_t *f, const tof_frame_t *frame);

/* True when a fusion step should run: every non-FAILED sensor has a fresh frame, or the oldest
 * fresh frame has waited for the fusion window. */
bool fusion_ready(const fusion_t *f, const sensor_health_t health[TOF_NUM_SENSORS], uint32_t now_us);

/* One fusion step over the fresh frames within the window; steps every cell once. */
void fusion_run(fusion_t *f, const sensor_health_t health[TOF_NUM_SENSORS], uint32_t now_us);

/* Map state from sensor health (MAP-11). */
map_state_t fusion_map_state(const sensor_health_t health[TOF_NUM_SENSORS]);

export_state_t fusion_export_state(const fusion_t *f, uint8_t sector, uint32_t now_us);

/* distances[72] in cm by 5.2: OCCUPIED -> distance, CLEAR_MEASURED -> max + 1, else 65535. */
void fusion_export(const fusion_t *f, uint32_t now_us, uint16_t distances[TOF_NUM_SECTORS]);

#endif /* CORE_FUSION_H */
