#ifndef CORE_FRAME_H
#define CORE_FRAME_H

#include <stdint.h>

#include "core/config.h"

/* Sensor frame handed from the driver to the core (SNS-06, SNS-09). Only the ULD outputs enabled by
 * SNS-06 are carried; field types match the VL53L8CX ULD result buffers. */

typedef struct {
    int16_t distance_mm;      /* along the zone ray (H-13) */
    uint16_t range_sigma_mm;
    uint32_t signal_per_spad;
    uint8_t target_status;
} tof_target_t;

typedef struct {
    uint8_t nb_target_detected; /* targets beyond TOF_TARGETS_PER_ZONE are ignored */
    uint32_t ambient_per_spad;
    tof_target_t target[TOF_TARGETS_PER_ZONE]; /* closest first (SNS-06) */
} tof_zone_t;

typedef struct {
    uint8_t sensor_id;   /* TOF_SENSOR_L/C/R */
    uint32_t frame_no;   /* driver frame counter */
    uint32_t t_ready_us; /* MCU time when the frame became ready */
    uint8_t res;         /* mode: matrix side, 4 or 8 */
    uint8_t freq_hz;     /* ranging frequency */
    tof_zone_t zone[TOF_MAX_ZONES]; /* raw ULD order, res * res entries used */
} tof_frame_t;

#endif /* CORE_FRAME_H */
