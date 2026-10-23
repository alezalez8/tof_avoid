#include "core/phase.h"

/* REQ: MAP-16 */
void phase_cfg_from_params(phase_cfg_t *cfg, const params_t *p)
{
    cfg->h_flight_m = params_get_f32(p, PARAM_PHASE_H_FLIGHT_M);
    cfg->h_hyst_m = params_get_f32(p, PARAM_PHASE_H_HYST_M);
    cfg->ground_policy = (uint8_t)params_get_u32(p, PARAM_GROUND_POLICY);
}

/* REQ: MAP-16 (PHASE_UNKNOWN until FC data arrive) */
void phase_init(phase_fsm_t *f, uint32_t now_us)
{
    f->phase = PHASE_UNKNOWN;
    f->last_known = PHASE_UNKNOWN;
    f->airborne = false;
    f->h_timer_on = false;
    f->t_h_since_us = now_us;
    f->t_enter_us = now_us;
}

static bool is_air(phase_t ph)
{
    return ph == PHASE_TAKEOFF || ph == PHASE_FLIGHT || ph == PHASE_LANDING;
}

/* IN_AIR before FLIGHT: TAKEOFF until h_tof >= PHASE_H_FLIGHT_M has held for 1 s; any sample below
 * the threshold restarts the confirmation (MAP-16a: no height hysteresis; PHASE_H_HYST_M is only the
 * operating rule of section 16). Without a valid height: FLIGHT at once (MAP-15b). */
/* REQ: MAP-16, MAP-16a, MAP-15b */
static phase_t in_air(phase_fsm_t *f, const phase_cfg_t *cfg, const phase_input_t *in, uint32_t now_us)
{
    if (!in->h_valid) {
        return PHASE_FLIGHT;
    }
    if (in->h_m >= cfg->h_flight_m) {
        if (!f->h_timer_on) {
            f->h_timer_on = true;
            f->t_h_since_us = now_us;
        }
    } else {
        f->h_timer_on = false;
    }
    if (f->h_timer_on && (uint32_t)(now_us - f->t_h_since_us) >= PHASE_FLIGHT_CONFIRM_US) {
        return PHASE_FLIGHT;
    }
    return PHASE_TAKEOFF;
}

/* Phase from fresh FC data, before the GROUND_RECOVER step. */
/* REQ: MAP-16, MAP-16a */
static phase_t next_known(phase_fsm_t *f, const phase_cfg_t *cfg, const phase_input_t *in, bool was_flight,
                          uint32_t now_us)
{
    if (was_flight && (in->landed_state == PHASE_LS_IN_AIR || in->landed_state == PHASE_LS_TAKEOFF)) {
        return PHASE_FLIGHT; /* MAP-16a: FLIGHT is left only on LS = LANDING / ON_GROUND or PHASE_UNKNOWN */
    }
    if (!in->armed) {
        return PHASE_GROUND;
    }
    switch (in->landed_state) {
    case PHASE_LS_ON_GROUND:
        return PHASE_ARMED_GROUND;
    case PHASE_LS_TAKEOFF:
        return PHASE_TAKEOFF;
    case PHASE_LS_LANDING:
        return PHASE_LANDING;
    case PHASE_LS_IN_AIR:
        return in_air(f, cfg, in, now_us);
    default:
        return PHASE_UNKNOWN; /* LS undefined */
    }
}

/* REQ: MAP-16 (scheme and table 5.4) */
uint8_t phase_update(phase_fsm_t *f, const phase_cfg_t *cfg, const phase_input_t *in, uint32_t now_us)
{
    const phase_t prev = f->phase;
    phase_t next;
    if (!in->hb_valid || !in->ess_valid) {
        next = PHASE_UNKNOWN;
    } else {
        /* MAP-16a: PHASE_UNKNOWN leaves FLIGHT; coming back, FLIGHT is entered again only by the entry
         * rule (1 s at PHASE_H_FLIGHT_M, or no valid height - MAP-15b). */
        const bool was_flight = prev == PHASE_FLIGHT;
        next = next_known(f, cfg, in, was_flight, now_us);
        if (!is_air(next) && next != PHASE_UNKNOWN && f->airborne && prev != PHASE_GROUND_RECOVER) {
            next = PHASE_GROUND_RECOVER; /* LS = ON_GROUND (or disarm) after a flight */
        }
    }
    if (next != PHASE_TAKEOFF) {
        f->h_timer_on = false;
    }
    if (is_air(next)) {
        f->airborne = true;
    }
    uint8_t evt = PHASE_EVT_NONE;
    if (next == PHASE_GROUND_RECOVER) {
        f->airborne = false;
        evt |= PHASE_EVT_RESET;
    }
    if (next != prev) {
        evt |= PHASE_EVT_CHANGED;
        f->t_enter_us = now_us;
        if (next == PHASE_UNKNOWN) {
            f->last_known = prev;
        }
    }
    f->phase = next;
    return evt;
}

/* REQ: MAP-17, MAP-07, MAP-07a */
bool phase_export_ground(phase_t ph, uint8_t ground_policy, bool height_valid)
{
    switch (ph) {
    case PHASE_FLIGHT:
#if TOF_ALLOW_GROUND_DROP
        /* DROP only in FLIGHT and only with a valid height (MAP-07). */
        return !(ground_policy == PARAM_GROUND_DROP && height_valid);
#else
        (void)ground_policy;
        (void)height_valid;
        return true; /* MAP-07a: DROP is not available in this build */
#endif
    case PHASE_UNKNOWN:
        return true; /* as in FLIGHT with KEEP */
    default:
        return false; /* ground phases, TAKEOFF, LANDING: 65535 */
    }
}

/* REQ: FLT-03, FLT-05 */
bool phase_hold_allowed(phase_t ph, bool mode_known)
{
    switch (ph) {
    case PHASE_TAKEOFF:
    case PHASE_FLIGHT:
    case PHASE_LANDING:
        return true;
    case PHASE_UNKNOWN:
        return mode_known;
    default:
        return false;
    }
}

/* REQ: DIA-04 */
bool phase_is_ground(phase_t ph)
{
    return ph == PHASE_GROUND || ph == PHASE_ARMED_GROUND || ph == PHASE_GROUND_RECOVER;
}

/* REQ: DIA-01 */
const char *phase_name(phase_t ph)
{
    static const char *const k_names[PHASE_COUNT] = {"UNK", "GND", "ARM", "TKO", "FLT", "LND", "RCV"};
    return ((unsigned)ph < (unsigned)PHASE_COUNT) ? k_names[ph] : "UNK";
}
