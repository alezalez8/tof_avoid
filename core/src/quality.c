#include "core/quality.h"

/* REQ: MAP-03 */
void quality_cfg_from_params(quality_cfg_t *cfg, const params_t *p)
{
    cfg->valid_status_mask = params_get_u32(p, PARAM_VALID_STATUS);
    cfg->sigma_max_mm = (uint16_t)params_get_u32(p, PARAM_SIGMA_MAX_MM);
    cfg->signal_min = params_get_u32(p, PARAM_SIGNAL_MIN);
}

/* REQ: MAP-03 */
bool quality_target_valid(const quality_cfg_t *cfg, const tof_target_t *t)
{
    if (t->target_status >= 32u || ((cfg->valid_status_mask >> t->target_status) & 1u) == 0u) {
        return false;
    }
    return t->range_sigma_mm <= cfg->sigma_max_mm && t->signal_per_spad >= cfg->signal_min;
}

/* REQ: MAP-03, SNS-06 */
void quality_zone_eval(const quality_cfg_t *cfg, const tof_zone_t *z, quality_zone_t *out)
{
    const uint8_t n = (z->nb_target_detected < TOF_TARGETS_PER_ZONE) ? z->nb_target_detected
                                                                      : (uint8_t)TOF_TARGETS_PER_ZONE;
    out->n_targets = n;
    out->n_valid = 0u;
    out->valid_mask = 0u;
    for (uint8_t k = 0; k < n; k++) {
        if (quality_target_valid(cfg, &z->target[k])) {
            out->n_valid++;
            out->valid_mask |= (uint8_t)(1u << k);
        }
    }
    out->has_target = n > 0u;
    out->zone_valid = out->n_valid > 0u; /* MAP-10a */
}
