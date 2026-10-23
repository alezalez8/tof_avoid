#include "core/fault_fsm.h"

#include <string.h>

#include "core/mavlink_tx.h"

/* REQ: FLT-03, FLT-08 */
void fault_cfg_from_params(fault_cfg_t *cfg, const params_t *p)
{
    cfg->action = (uint8_t)params_get_u32(p, PARAM_FAULT_ACTION);
    cfg->trigger_mask = (uint8_t)params_get_u32(p, PARAM_FAULT_TRIGGER_MASK);
    cfg->on_reboot = params_get_u32(p, PARAM_FAULT_ON_REBOOT) != 0u;
}

/* REQ: FLT-04 */
void fault_init(fault_fsm_t *f)
{
    memset(f, 0, sizeof *f);
}

/* REQ: FLT-04, MAP-16 (GROUND_RECOVER) */
void fault_reset_episode(fault_fsm_t *f)
{
    f->episode = false;
    f->reboot_latch = false;
    f->series = FAULT_SERIES_NONE;
    f->sends = 0u;
}

/* REQ: FLT-03 */
void fault_send_failed(fault_fsm_t *f)
{
    if (f->sends > 0u) {
        f->sends--;
        f->sends_total--;
        f->t_last_send_us -= FAULT_RETRY_US; /* due again at once */
    }
}

/* REQ: FLT-03, FLT-05 */
bool fault_is_auto_mode(uint32_t custom_mode)
{
    return custom_mode == TOF_COPTER_MODE_AUTO || custom_mode == TOF_COPTER_MODE_GUIDED ||
           custom_mode == TOF_COPTER_MODE_RTL;
}

static void send(fault_fsm_t *f, uint32_t now_us, fault_out_t *out)
{
    out->send = true;
    if (f->sends == 0u) {
        out->events |= FAULT_EVT_HOLD_SENT; /* first transmission of the series */
    }
    out->mode = f->target_mode;
    out->confirmation = f->sends;
    f->sends++;
    f->sends_total++;
    f->t_last_send_us = now_us;
}

/* REQ: FLT-03, FLT-04, FLT-06 */
static void run_series(fault_fsm_t *f, const fault_cfg_t *cfg, const fault_fc_t *fc, bool is_auto,
                       uint32_t now_us, fault_out_t *out)
{
    /* FLT-03, FLT-05: armed, mode in {AUTO, GUIDED, RTL}, allowed phase, FAULT_ACTION != NONE. */
    const bool allowed = cfg->action != PARAM_FAULT_NONE && fc->mode_known && fc->armed && is_auto &&
                         phase_hold_allowed(fc->phase, fc->mode_known);
    switch (f->series) {
    case FAULT_SERIES_NONE:
        if (allowed) {
            f->series = FAULT_SERIES_SENDING;
            f->sends = 0u;
            f->hb_at_first_send = fc->hb_count;
            f->target_mode = (cfg->action == PARAM_FAULT_LOITER) ? TOF_COPTER_MODE_LOITER : TOF_COPTER_MODE_BRAKE;
            send(f, now_us, out); /* on the update that detected the fault: <= 100 ms */
        } else if (f->reboot_latch) {
            f->reboot_latch = false; /* FLT-08 applies only to the first HEARTBEAT */
        } else {
            /* waiting for the conditions */
        }
        break;
    case FAULT_SERIES_SENDING:
        /* Success only by the mode in a HEARTBEAT received after the first command (not COMMAND_ACK). */
        if (fc->mode_known && fc->hb_count != f->hb_at_first_send && fc->custom_mode == f->target_mode) {
            f->series = FAULT_SERIES_SUCCEEDED;
            out->events |= FAULT_EVT_HOLD_OK;
        } else if (fc->mode_known && fc->hb_count != f->hb_at_first_send && (!fc->armed || !is_auto)) {
            f->series = FAULT_SERIES_ABORTED; /* mode changed by the pilot or the FC, or disarmed */
        } else if ((uint32_t)(now_us - f->t_last_send_us) < FAULT_RETRY_US) {
            /* waiting */
        } else if (f->sends >= FAULT_MAX_SENDS) {
            f->series = FAULT_SERIES_REJECTED;
            out->events |= FAULT_EVT_HOLD_REJECTED;
        } else if (allowed) {
            send(f, now_us, out);
        } else {
            /* FC link or phase does not allow sending now: paused */
        }
        break;
    default:
        /* SUCCEEDED / REJECTED / ABORTED: no more commands in this episode (FLT-04). */
        if (fc->armed && is_auto && !f->was_auto) {
            out->events |= FAULT_EVT_PILOT_RESUMED;
        }
        break;
    }
    if (f->series >= FAULT_SERIES_SUCCEEDED) {
        f->reboot_latch = false;
    }
}

/* REQ: FLT-01 (classes), FLT-03, FLT-04, FLT-05, FLT-06, FLT-08 */
void fault_update(fault_fsm_t *f, const fault_cfg_t *cfg, uint8_t active, const fault_fc_t *fc, uint32_t now_us,
                  fault_out_t *out)
{
    memset(out, 0, sizeof *out);
    f->active = active;
    const bool is_auto = fc->mode_known && fault_is_auto_mode(fc->custom_mode);
    if (fc->first_hb && fc->armed && fault_is_auto_mode(fc->custom_mode)) {
        /* FLT-08: the module restarted in flight. */
        out->events |= FAULT_EVT_REBOOTED_IN_FLIGHT;
        f->reboot_latch = cfg->on_reboot;
    }
    const bool now_episode = (active & cfg->trigger_mask) != 0u || f->reboot_latch;
    if (now_episode && !f->episode) {
        f->episode = true;
        f->series = FAULT_SERIES_NONE;
        f->sends = 0u;
        f->episodes_total++;
        out->events |= FAULT_EVT_EPISODE_START;
    }
    if (f->episode && now_episode) {
        run_series(f, cfg, fc, is_auto, now_us, out); /* may drop the reboot latch */
    }
    if (f->episode && (active & cfg->trigger_mask) == 0u && !f->reboot_latch) {
        fault_reset_episode(f); /* FLT-04: the episode lasts until the fault is gone */
        out->events |= FAULT_EVT_EPISODE_END;
    }
    f->was_auto = fc->armed && is_auto;
}

/* REQ: FLT-10 */
void fault_wall_apply(const fusion_t *fu, const sensor_health_t health[TOF_NUM_SENSORS], uint16_t min_cm,
                      uint16_t distances[TOF_NUM_SECTORS])
{
    for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
        if (health[s] != SENSOR_FAILED) {
            continue;
        }
        for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
            if (fu->sector_zones[k][s] != 0u) {
                distances[k] = min_cm;
            }
        }
    }
}
