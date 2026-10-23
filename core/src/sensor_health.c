#include "core/sensor_health.h"

#include <string.h>

/* REQ: SNS-11, MAP-10 */
void sh_cfg_from_params(sh_cfg_t *cfg, const params_t *p)
{
    cfg->period_us = 1000000u / params_get_u32(p, PARAM_SENSOR_FREQ_HZ);
    cfg->valid_frac_min = params_get_f32(p, PARAM_VALID_FRAC_MIN);
    cfg->degraded_time_us = params_get_u32(p, PARAM_DEGRADED_TIME_MS) * 1000u;
}

/* REQ: SNS-11 */
void sh_init(sh_t *s, uint32_t now_us)
{
    memset(s, 0, sizeof *s);
    s->sup = SH_RUNNING;
    s->health = SENSOR_OK;
    s->t_last_frame_us = now_us;
}

/* Data stopped at the last frame: counts as zero valid zones from then on (MAP-10). */
static void mark_no_data(sh_t *s)
{
    if (!s->low_active) {
        s->low_active = true;
        s->t_low_since_us = s->t_last_frame_us;
    }
}

static void request_reinit(sh_t *s, uint32_t now_us)
{
    s->sup = SH_REINIT;
    s->reinit_request = true;
    s->t_reinit_us = now_us;
    s->consec_errors = 0u;
    s->reinit_total++;
}

/* One reinit attempt failed: retry at once, or FAILED after SH_REINIT_FAILS_TO_FAILED in a row. */
/* REQ: SNS-11 */
static void reinit_failed(sh_t *s, uint32_t now_us)
{
    s->reinit_unconfirmed = false;
    if (s->consec_reinit_fails < 0xFFu) {
        s->consec_reinit_fails++;
    }
    if (s->consec_reinit_fails >= SH_REINIT_FAILS_TO_FAILED) {
        s->sup = SH_FAILED;
        s->reinit_request = false;
    } else {
        request_reinit(s, now_us);
    }
}

/* Error or missing frames while running. */
/* REQ: SNS-11 */
static void trigger(sh_t *s, uint32_t now_us)
{
    mark_no_data(s);
    if (s->reinit_unconfirmed) {
        reinit_failed(s, now_us); /* the last reinit never produced a frame */
    } else {
        request_reinit(s, now_us);
    }
}

/* REQ: SNS-11, MAP-10 */
void sh_on_frame(sh_t *s, const sh_cfg_t *cfg, float valid_frac, uint32_t now_us)
{
    s->sup = SH_RUNNING;
    s->reinit_request = false;
    s->reinit_unconfirmed = false;
    s->consec_errors = 0u;
    s->consec_reinit_fails = 0u;
    s->t_last_frame_us = now_us;
    if (valid_frac < cfg->valid_frac_min) {
        if (!s->low_active) {
            s->low_active = true;
            s->t_low_since_us = now_us;
        }
    } else {
        s->low_active = false;
    }
}

/* REQ: SNS-11 (3 errors in a row) */
void sh_on_error(sh_t *s, uint32_t now_us)
{
    if (s->sup != SH_RUNNING) {
        return;
    }
    s->consec_errors++;
    if (s->consec_errors >= SH_ERRORS_TO_REINIT) {
        trigger(s, now_us);
    }
}

/* REQ: SNS-11 */
bool sh_take_reinit_request(sh_t *s)
{
    const bool r = s->reinit_request;
    s->reinit_request = false;
    return r;
}

/* REQ: SNS-11 */
void sh_on_reinit_done(sh_t *s, bool ok, uint32_t now_us)
{
    if (s->sup != SH_REINIT) {
        return;
    }
    if (ok) {
        /* Confirmed only by the first frame; until then the 3-period check runs again. */
        s->sup = SH_RUNNING;
        s->reinit_unconfirmed = true;
        s->t_last_frame_us = now_us;
        return;
    }
    if (s->consec_reinit_fails >= SH_REINIT_FAILS_TO_FAILED) {
        s->sup = SH_FAILED; /* a retry from FAILED failed: wait for the next 10 s slot */
        return;
    }
    reinit_failed(s, now_us);
}

/* REQ: SNS-11, MAP-10 */
void sh_update(sh_t *s, const sh_cfg_t *cfg, uint32_t now_us)
{
    switch (s->sup) {
    case SH_RUNNING:
        if ((uint32_t)(now_us - s->t_last_frame_us) > SH_PERIODS_TO_REINIT * cfg->period_us) {
            if (s->reinit_unconfirmed && s->consec_reinit_fails >= SH_REINIT_FAILS_TO_FAILED) {
                s->reinit_unconfirmed = false;
                s->sup = SH_FAILED; /* a retry from FAILED gave no frame */
            } else {
                trigger(s, now_us);
            }
        }
        break;
    case SH_REINIT:
        if ((uint32_t)(now_us - s->t_reinit_us) > SH_REINIT_TIMEOUT_US) {
            if (s->consec_reinit_fails >= SH_REINIT_FAILS_TO_FAILED) {
                s->sup = SH_FAILED;
                s->reinit_request = false;
            } else {
                reinit_failed(s, now_us);
            }
        }
        break;
    default: /* SH_FAILED */
        if ((uint32_t)(now_us - s->t_reinit_us) >= SH_FAILED_RETRY_US) {
            request_reinit(s, now_us);
        }
        break;
    }

    if (s->sup == SH_FAILED) {
        s->health = SENSOR_FAILED;
    } else if (s->low_active && (uint32_t)(now_us - s->t_low_since_us) > cfg->degraded_time_us) {
        s->health = SENSOR_DEGRADED;
    } else {
        s->health = SENSOR_OK;
    }
}
