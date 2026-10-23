#include "core/events.h"

#include <string.h>

#include "core/version.h"

/* Message ids for the per-type rate limit: channel * 4 + state, then 32 + one-shot. */
#define EV_ID_ONESHOT_BASE 32u
#define EV_ID_COUNT EV_MSG_IDS

/* REQ: DIA-04, DIA-05 */
void ev_cfg_from_params(ev_cfg_t *cfg, const params_t *p)
{
    cfg->warn_cm = (uint16_t)params_get_u32(p, PARAM_WARN_CM);
    cfg->crit_cm = (uint16_t)params_get_u32(p, PARAM_CRIT_CM);
    cfg->hyst_cm = (uint16_t)params_get_u32(p, PARAM_WARN_HYST_CM);
}

/* REQ: DIA-03 */
void ev_init(ev_t *e)
{
    memset(e, 0, sizeof *e);
}

/* REQ: DIA-04 (state channels) */
void ev_set_state(ev_t *e, ev_channel_t ch, uint8_t state)
{
    if ((unsigned)ch < (unsigned)EV_CH_COUNT) {
        e->cur[ch] = state;
    }
}

/* REQ: DIA-04 (deduplication: a pending one-shot is raised once) */
void ev_raise(ev_t *e, ev_oneshot_t ev, uint32_t value)
{
    if ((unsigned)ev < (unsigned)EV_ONE_COUNT) {
        e->pending[ev] = true;
        e->one_value[ev] = value;
    }
}

/* REQ: DIA-04 (WARN_HYST_CM, ground phases: real obstacles only) */
void ev_obstacle(ev_t *e, const ev_cfg_t *cfg, phase_t ph, uint16_t exported_cm, int16_t exported_az,
                 uint16_t real_cm, int16_t real_az)
{
    const bool ground = phase_is_ground(ph);
    const uint16_t d = ground ? real_cm : exported_cm;
    const int16_t az = ground ? real_az : exported_az;
    const uint8_t was = e->cur[EV_CH_OBSTACLE];
    uint8_t lvl = 0u;
    if (d == EV_DIST_NONE) {
        lvl = 0u;
    } else if (d <= cfg->crit_cm || (was == 2u && d <= (uint32_t)cfg->crit_cm + cfg->hyst_cm)) {
        lvl = 2u;
    } else if (d <= cfg->warn_cm || (was >= 1u && d <= (uint32_t)cfg->warn_cm + cfg->hyst_cm)) {
        lvl = 1u;
    } else {
        lvl = 0u;
    }
    e->cur[EV_CH_OBSTACLE] = lvl;
    if (lvl > 0u) {
        e->obst_cm = d;
        e->obst_az_deg = az;
    }
}

/* ---- text building without printf (no floating point formatting) ---- */

typedef struct {
    char *s;
    uint8_t n;
} sb_t;

static void sb_str(sb_t *b, const char *t)
{
    while (*t != '\0' && b->n < EV_TEXT_LEN) {
        b->s[b->n++] = *t++;
    }
    b->s[b->n] = '\0';
}

static void sb_uint(sb_t *b, uint32_t v, uint8_t min_digits)
{
    char d[10];
    uint8_t k = 0u;
    do {
        d[k++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v > 0u && k < sizeof d);
    while (k < min_digits && k < sizeof d) {
        d[k++] = '0';
    }
    while (k > 0u && b->n < EV_TEXT_LEN) {
        b->s[b->n++] = d[--k];
    }
    b->s[b->n] = '\0';
}

static void sb_int(sb_t *b, int32_t v)
{
    if (v < 0) {
        sb_str(b, "-");
        sb_uint(b, (uint32_t)(-v), 1u);
    } else {
        sb_uint(b, (uint32_t)v, 1u);
    }
}

static const char k_sensor_name[3][2] = {"L", "C", "R"};

/* REQ: DIA-03 (texts and severities) */
static uint8_t channel_text(const ev_t *e, uint8_t ch, uint8_t st, sb_t *b)
{
    switch (ch) {
    case EV_CH_OBSTACLE:
        if (st == 0u) {
            sb_str(b, "TOF OBST CLEAR");
            return EV_SEV_INFO;
        }
        sb_str(b, (st == 2u) ? "TOF OBST CRIT " : "TOF OBST WARN ");
        sb_uint(b, e->obst_cm / 100u, 1u);
        sb_str(b, ".");
        sb_uint(b, e->obst_cm % 100u, 2u);
        sb_str(b, "m ");
        sb_int(b, e->obst_az_deg);
        sb_str(b, "deg");
        return (st == 2u) ? EV_SEV_CRITICAL : EV_SEV_WARNING;
    case EV_CH_SENSOR_L:
    case EV_CH_SENSOR_C:
    case EV_CH_SENSOR_R:
        sb_str(b, "TOF SENSOR ");
        sb_str(b, k_sensor_name[ch - EV_CH_SENSOR_L]);
        sb_str(b, (st != 0u) ? " LOST" : " OK");
        return (st != 0u) ? EV_SEV_CRITICAL : EV_SEV_INFO;
    case EV_CH_STALE:
        sb_str(b, "TOF DATA STALE");
        return EV_SEV_WARNING;
    case EV_CH_MAP_LOST:
        sb_str(b, "TOF MAP LOST");
        return EV_SEV_CRITICAL;
    case EV_CH_FC_LINK:
        sb_str(b, "TOF FC LINK LOST");
        return EV_SEV_WARNING;
    default: /* EV_CH_PHASE */
        sb_str(b, "TOF PHASE UNKNOWN");
        return EV_SEV_WARNING;
    }
}

/* REQ: DIA-03, SNS-12a, SW-07, FLT-04, FLT-06, FLT-08 */
static uint8_t oneshot_text(const ev_t *e, uint8_t k, sb_t *b)
{
    switch (k) {
    case EV_ONE_HOLD_SENT:
        sb_str(b, (e->one_value[k] == 5u) ? "TOF HOLD SENT LOITER" : "TOF HOLD SENT BRAKE");
        return EV_SEV_CRITICAL;
    case EV_ONE_HOLD_REJECTED:
        sb_str(b, "TOF HOLD REJECTED");
        return EV_SEV_CRITICAL;
    case EV_ONE_REBOOTED:
        sb_str(b, "TOF REBOOTED IN FLIGHT");
        return EV_SEV_CRITICAL;
    case EV_ONE_PILOT_RESUMED:
        sb_str(b, "TOF FAULT ACTIVE, AUTO WITHOUT HOLD");
        return EV_SEV_WARNING;
    case EV_ONE_PARAMS_DEFAULT:
        sb_str(b, "TOF PARAMS DEFAULT");
        return EV_SEV_WARNING;
    case EV_ONE_ORIENT:
        sb_str(b, "TOF ORIENT NOT CONFIRMED");
        return EV_SEV_WARNING;
    default: /* EV_ONE_READY */
        sb_str(b, "TOF READY ");
        sb_str(b, tof_version_string());
        sb_str(b, " ");
        sb_str(b, tof_version_git_hash());
        return EV_SEV_INFO;
    }
}

/* Channels whose return to 0 is not reported. */
static bool silent_clear(uint8_t ch)
{
    return ch == EV_CH_STALE || ch == EV_CH_MAP_LOST || ch == EV_CH_FC_LINK || ch == EV_CH_PHASE;
}

static bool type_allowed(const ev_t *e, uint8_t id, uint32_t now_us)
{
    return !e->sent_any[id] || (uint32_t)(now_us - e->t_sent[id]) >= EV_RATE_TYPE_US;
}

/* REQ: DIA-03, DIA-04 (state change only, dedup, <= 1/s per type, <= 3/s total) */
bool ev_next_statustext(ev_t *e, uint32_t now_us, uint8_t *severity, char text[EV_TEXT_LEN + 1u])
{
    for (uint8_t ch = 0; ch < EV_CH_COUNT; ch++) {
        if (silent_clear(ch) && e->cur[ch] == 0u) {
            e->rep[ch] = 0u;
        }
    }
    if (e->n_window >= EV_RATE_TOTAL && (uint32_t)(now_us - e->t_window[0]) < EV_RATE_TYPE_US) {
        return false; /* 3 messages in the last second */
    }
    /* Highest severity first (lowest MAV_SEVERITY number), then table order. */
    uint8_t best_id = EV_ID_COUNT;
    uint8_t best_sev = 0xFFu;
    char best_text[EV_TEXT_LEN + 1u];
    for (uint8_t ch = 0; ch < EV_CH_COUNT; ch++) {
        if (e->cur[ch] == e->rep[ch]) {
            continue;
        }
        const uint8_t id = (uint8_t)(ch * 4u + e->cur[ch]);
        char t[EV_TEXT_LEN + 1u];
        sb_t b = {t, 0u};
        const uint8_t sev = channel_text(e, ch, e->cur[ch], &b);
        if (type_allowed(e, id, now_us) && sev < best_sev) {
            best_sev = sev;
            best_id = id;
            memcpy(best_text, t, sizeof best_text);
        }
    }
    for (uint8_t k = 0; k < EV_ONE_COUNT; k++) {
        if (!e->pending[k]) {
            continue;
        }
        const uint8_t id = (uint8_t)(EV_ID_ONESHOT_BASE + k);
        char t[EV_TEXT_LEN + 1u];
        sb_t b = {t, 0u};
        const uint8_t sev = oneshot_text(e, k, &b);
        if (type_allowed(e, id, now_us) && sev < best_sev) {
            best_sev = sev;
            best_id = id;
            memcpy(best_text, t, sizeof best_text);
        }
    }
    if (best_id == EV_ID_COUNT) {
        return false;
    }
    if (best_id >= EV_ID_ONESHOT_BASE) {
        e->pending[best_id - EV_ID_ONESHOT_BASE] = false;
    } else {
        e->rep[best_id / 4u] = (uint8_t)(best_id % 4u);
    }
    e->sent_any[best_id] = true;
    e->t_sent[best_id] = now_us;
    if (e->n_window >= EV_RATE_TOTAL) {
        e->t_window[0] = e->t_window[1];
        e->t_window[1] = e->t_window[2];
        e->n_window--;
    }
    e->t_window[e->n_window++] = now_us;
    e->sent_total++;
    *severity = best_sev;
    memcpy(text, best_text, EV_TEXT_LEN + 1u);
    ev_log(e, now_us, EV_LOG_STATUSTEXT, best_sev, best_id);
    return true;
}

/* REQ: MAP-13 (log record of hold expiry), DIA-06 */
void ev_log(ev_t *e, uint32_t t_us, ev_log_kind_t kind, uint8_t a, uint16_t b)
{
    if (e->log_count >= EV_LOG_LEN) {
        e->log_dropped++; /* DIA-06: dropped with a counter */
        return;
    }
    ev_log_t *r = &e->log[(e->log_head + e->log_count) % EV_LOG_LEN];
    r->t_us = t_us;
    r->kind = (uint8_t)kind;
    r->a = a;
    r->b = b;
    e->log_count++;
}

/* REQ: DIA-06 */
bool ev_take_log(ev_t *e, ev_log_t *out)
{
    if (e->log_count == 0u) {
        return false;
    }
    *out = e->log[e->log_head];
    e->log_head = (uint8_t)((e->log_head + 1u) % EV_LOG_LEN);
    e->log_count--;
    return true;
}
