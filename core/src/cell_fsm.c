#include "core/cell_fsm.h"

#include <string.h>

/* REQ: MAP-13 */
void cell_cfg_from_params(cell_cfg_t *cfg, const params_t *p)
{
    cfg->clear_confirm_frames = (uint8_t)params_get_u32(p, PARAM_CLEAR_CONFIRM_FRAMES);
    cfg->dist_relax_frames = (uint8_t)params_get_u32(p, PARAM_DIST_RELAX_FRAMES);
    cfg->hold_max_us = params_get_u32(p, PARAM_HOLD_MAX_MS) * 1000u;
    cfg->clear_enabled = params_get_u32(p, PARAM_AMBIENT_CLEAR_MAX) > 0u;
}

/* REQ: MAP-13 */
void cell_init(cell_t *c)
{
    memset(c, 0, sizeof *c);
    c->state = CELL_UNKNOWN;
}

static uint8_t sat_inc(uint8_t v)
{
    return (v < 0xFFu) ? (uint8_t)(v + 1u) : v;
}

static void take_obstacle(cell_t *c, uint16_t dist_cm, uint8_t sensor, uint8_t zone, bool angle_ok,
                          bool ground)
{
    c->dist_cm = dist_cm;
    c->src_sensor = sensor;
    c->src_zone = zone;
    c->angle_ok = angle_ok;
    c->ground_suspect = ground;
    c->relax_count = 0u;
}

/* REQ: MAP-13 */
static uint8_t step_occ(cell_t *c, const cell_cfg_t *cfg, const cell_input_t *in, uint32_t now_us)
{
    uint8_t evt = CELL_EVT_NONE;
    c->clr_count = 0u;
    /* An older frame of another sensor must not move the hold timer backwards. */
    if (c->state != CELL_OCCUPIED || (int32_t)(now_us - c->t_valid_us) > 0) {
        c->t_valid_us = now_us;
    }
    if (c->state != CELL_OCCUPIED || in->dist_cm <= c->dist_cm) {
        /* Any state + OCC closer than the held distance -> OCCUPIED at once. */
        if (c->state != CELL_OCCUPIED) {
            evt = CELL_EVT_OCCUPIED;
        }
        c->state = CELL_OCCUPIED;
        take_obstacle(c, in->dist_cm, in->src_sensor, in->src_zone, in->angle_ok, in->ground_suspect);
        return evt;
    }
    /* OCCUPIED + OCC farther: grow after DIST_RELAX_FRAMES confirmations in a row,
     * to the nearest distance seen during the run. */
    if (c->relax_count == 0u || in->dist_cm < c->relax_min_cm) {
        c->relax_min_cm = in->dist_cm;
        c->relax_src_sensor = in->src_sensor;
        c->relax_src_zone = in->src_zone;
        c->relax_angle_ok = in->angle_ok;
        c->relax_ground = in->ground_suspect;
    }
    c->relax_count = sat_inc(c->relax_count);
    if (c->relax_count >= cfg->dist_relax_frames) {
        take_obstacle(c, c->relax_min_cm, c->relax_src_sensor, c->relax_src_zone, c->relax_angle_ok,
                      c->relax_ground);
    }
    return evt;
}

/* REQ: MAP-13 */
static uint8_t step_clr(cell_t *c, const cell_cfg_t *cfg)
{
    c->relax_count = 0u;
    c->clr_count = sat_inc(c->clr_count);
    if (c->clr_count < cfg->clear_confirm_frames) {
        return CELL_EVT_NONE;
    }
    if (c->state == CELL_OCCUPIED) {
        /* OCCUPIED + CLR confirmed -> CLEAR_MEASURED if enabled, otherwise UNKNOWN. */
        c->state = cfg->clear_enabled ? CELL_CLEAR_MEASURED : CELL_UNKNOWN;
        return CELL_EVT_RELEASED;
    }
    if (c->state == CELL_UNKNOWN && cfg->clear_enabled) {
        c->state = CELL_CLEAR_MEASURED;
    }
    return CELL_EVT_NONE;
}

/* REQ: MAP-13 */
static uint8_t step_inv(cell_t *c, const cell_cfg_t *cfg, uint32_t now_us)
{
    c->relax_count = 0u;
    c->clr_count = 0u;
    if (c->state != CELL_OCCUPIED) {
        /* UNKNOWN / CLEAR_MEASURED + INV -> UNKNOWN. */
        c->state = CELL_UNKNOWN;
        return CELL_EVT_NONE;
    }
    /* OCCUPIED + INV: hold; older than HOLD_MAX_MS -> UNKNOWN (wrap-safe age). */
    if ((uint32_t)(now_us - c->t_valid_us) > cfg->hold_max_us) {
        c->state = CELL_UNKNOWN;
        return CELL_EVT_HOLD_EXPIRED;
    }
    return CELL_EVT_NONE;
}

/* REQ: MAP-13 (table 5.3) */
uint8_t cell_fsm_step(cell_t *c, const cell_cfg_t *cfg, const cell_input_t *in, uint32_t now_us)
{
    switch (in->kind) {
    case CELL_CAND_OCC:
        return step_occ(c, cfg, in, now_us);
    case CELL_CAND_CLR:
        return step_clr(c, cfg);
    default:
        return step_inv(c, cfg, now_us);
    }
}
