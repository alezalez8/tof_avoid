#ifndef CORE_TOF_CORE_H
#define CORE_TOF_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "core/config.h"
#include "core/frame.h"
#include "core/fusion.h"
#include "core/ground.h"
#include "core/params.h"
#include "core/sensor_health.h"

/* Core API: the app hands in sensor frames (SNS-09) and the current time; the core keeps sensor
 * health (SNS-11, MAP-10) and the fused map (MAP-05...MAP-13). Time is always an argument. */

typedef enum {
    TOF_PUSH_OK = 0,
    TOF_PUSH_BAD_SENSOR, /* unknown sensor id: ignored */
    TOF_PUSH_BAD_MODE    /* resolution or frequency differs from the parameters: counted as error */
} tof_push_status_t;

typedef struct {
    uint32_t frames;   /* accepted frames */
    uint32_t gaps;     /* frame_no jumps (lost frames) */
    uint32_t rejected; /* TOF_PUSH_BAD_MODE */
} tof_sensor_stats_t;

typedef struct {
    params_t params;
    sh_cfg_t sh_cfg;
    sh_t sh[TOF_NUM_SENSORS];
    fusion_t fusion;
    uint8_t freq_hz;
    bool map_pending; /* a fusion step ran inside push_frame; reported by the next update */
    bool orient_not_confirmed_event; /* SNS-12a: ORIENT_CONFIRMED = 0 at start; cleared by the app */
    bool have_frame_no[TOF_NUM_SENSORS];
    uint32_t last_frame_no[TOF_NUM_SENSORS];
    uint32_t t_push_us[TOF_NUM_SENSORS]; /* MAP-12 t2: frame handed to the core */
    tof_sensor_stats_t stats[TOF_NUM_SENSORS];
} tof_core_t;

/* p must pass params_validate(). Ranging is assumed to start at now_us. */
void tof_core_init(tof_core_t *c, const params_t *p, uint32_t now_us);

/* g: ground context at the frame time (MAP-14); NULL or not valid - no target is ground-compatible. */
tof_push_status_t tof_core_push_frame(tof_core_t *c, const tof_frame_t *frame, const ground_ctx_t *g,
                                      uint32_t now_us);

/* The driver reports a failed transaction or read for this sensor. */
void tof_core_sensor_error(tof_core_t *c, uint8_t sensor, uint32_t now_us);

/* SNS-11 reinit handshake with the driver. */
bool tof_core_take_reinit_request(tof_core_t *c, uint8_t sensor);
void tof_core_reinit_done(tof_core_t *c, uint8_t sensor, bool ok, uint32_t now_us);

/* Health timers and, when ready, one fusion step. True if a new fused map was produced. */
bool tof_core_update(tof_core_t *c, uint32_t now_us);

sensor_health_t tof_core_sensor_health(const tof_core_t *c, uint8_t sensor);
map_state_t tof_core_map_state(const tof_core_t *c);

/* OBSTACLE_DISTANCE distances by 5.2. */
void tof_core_export(const tof_core_t *c, uint32_t now_us, uint16_t distances[TOF_NUM_SECTORS]);

#endif /* CORE_TOF_CORE_H */
