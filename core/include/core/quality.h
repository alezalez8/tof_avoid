#ifndef CORE_QUALITY_H
#define CORE_QUALITY_H

#include <stdbool.h>
#include <stdint.h>

#include "core/frame.h"
#include "core/params.h"

/* Target validity (MAP-03): status in VALID_STATUS, sigma <= SIGMA_MAX_MM, signal >= SIGNAL_MIN. */

typedef struct {
    uint32_t valid_status_mask; /* bit s set: target_status s is valid (s < 32) */
    uint16_t sigma_max_mm;
    uint32_t signal_min;
} quality_cfg_t;

typedef struct {
    uint8_t n_targets;  /* min(nb_target_detected, TOF_TARGETS_PER_ZONE) */
    uint8_t n_valid;
    uint8_t valid_mask; /* bit k: target k is valid */
    bool has_target;    /* at least one reported target */
    bool zone_valid;    /* has_target and at least one valid target (MAP-10a) */
} quality_zone_t;

void quality_cfg_from_params(quality_cfg_t *cfg, const params_t *p);

bool quality_target_valid(const quality_cfg_t *cfg, const tof_target_t *t);

/* Classifies every reported target of the zone separately (up to TOF_TARGETS_PER_ZONE). */
void quality_zone_eval(const quality_cfg_t *cfg, const tof_zone_t *z, quality_zone_t *out);

#endif /* CORE_QUALITY_H */
