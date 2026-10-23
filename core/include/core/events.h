#ifndef CORE_EVENTS_H
#define CORE_EVENTS_H

#include <stdbool.h>
#include <stdint.h>

#include "core/params.h"
#include "core/phase.h"

/* STATUSTEXT events (DIA-03) under the rules of DIA-04, and event records for the debug log
 * (DIA-06: MAP-13 hold expiry, COMMAND_ACK, phase changes, ...).
 *
 * DIA-04: a message only on a change of state; obstacle levels with WARN_HYST_CM hysteresis;
 * repeated raises of a pending event are merged (deduplication); <= 1 message/s per message type and
 * <= 3 messages/s in total - a held-back change is sent later if the state still differs from the
 * last reported one. In GROUND / ARMED_GROUND obstacle warnings use real obstacles only, never
 * GROUND_SUSPECT. STATUSTEXT never carries distances for control: the FC takes them only from
 * OBSTACLE_DISTANCE. All texts start with "TOF" and are <= 50 characters. */

/* MAV_SEVERITY values. */
#define EV_SEV_CRITICAL 2u
#define EV_SEV_WARNING 4u
#define EV_SEV_INFO 6u

#define EV_TEXT_LEN 50u
#define EV_RATE_TYPE_US 1000000u /* per message type */
#define EV_RATE_TOTAL 3u         /* messages per 1 s window */
#define EV_DIST_NONE 65535u      /* no obstacle */
#define EV_MSG_IDS 64u           /* message ids: channel * 4 + state, 32 + one-shot */

/* State channels: a message when the state differs from the last reported one. */
typedef enum {
    EV_CH_OBSTACLE = 0, /* 0 none, 1 WARN, 2 CRIT */
    EV_CH_SENSOR_L,     /* 0 ok, 1 lost (FAILED) */
    EV_CH_SENSOR_C,
    EV_CH_SENSOR_R,
    EV_CH_STALE,        /* 1: no fused map for 2 x VALID_DATA_MAX_AGE_MS; back to 0 silently */
    EV_CH_MAP_LOST,     /* 1: map LOST; back to 0 silently */
    EV_CH_FC_LINK,      /* 1: FC telemetry lost; back to 0 silently */
    EV_CH_PHASE,        /* 1: PHASE_UNKNOWN; back to 0 silently */
    EV_CH_COUNT
} ev_channel_t;

/* One-shot events. */
typedef enum {
    EV_ONE_HOLD_SENT = 0,  /* value: custom mode */
    EV_ONE_HOLD_REJECTED,  /* FLT-06 */
    EV_ONE_REBOOTED,       /* FLT-08 */
    EV_ONE_PILOT_RESUMED,  /* FLT-04 */
    EV_ONE_PARAMS_DEFAULT, /* SW-07: both slots invalid */
    EV_ONE_ORIENT,         /* SNS-12a */
    EV_ONE_READY,          /* start-up readiness with version */
    EV_ONE_COUNT
} ev_oneshot_t;

/* Debug log record kinds. */
typedef enum {
    EV_LOG_HOLD_EXPIRED = 1, /* MAP-13: a = view (0 all, 1 real), b = sector */
    EV_LOG_STATUSTEXT,       /* a = severity, b = message id */
    EV_LOG_COMMAND_ACK,      /* FLT-03: a = result, b = command */
    EV_LOG_HOLD_COMMAND,     /* a = confirmation, b = custom mode */
    EV_LOG_PHASE,            /* a = new phase, b = previous phase */
    EV_LOG_FAULT,            /* a = active classes, b = 1 start / 0 end */
    EV_LOG_NACK              /* ICD-04: b = refused command */
} ev_log_kind_t;

typedef struct {
    uint32_t t_us;
    uint8_t kind;
    uint8_t a;
    uint16_t b;
} ev_log_t;

#define EV_LOG_LEN 32u

typedef struct {
    uint16_t warn_cm;
    uint16_t crit_cm;
    uint16_t hyst_cm;
} ev_cfg_t;

typedef struct {
    uint8_t cur[EV_CH_COUNT];
    uint8_t rep[EV_CH_COUNT];
    bool pending[EV_ONE_COUNT];
    uint32_t one_value[EV_ONE_COUNT];
    uint16_t obst_cm;  /* distance and azimuth of the obstacle for the message text */
    int16_t obst_az_deg;
    bool sent_any[EV_MSG_IDS]; /* per message id */
    uint32_t t_sent[EV_MSG_IDS];
    uint32_t t_window[EV_RATE_TOTAL]; /* times of the last 3 messages */
    uint8_t n_window;
    uint32_t sent_total;

    ev_log_t log[EV_LOG_LEN];
    uint8_t log_head;
    uint8_t log_count;
    uint32_t log_dropped;
} ev_t;

void ev_cfg_from_params(ev_cfg_t *cfg, const params_t *p);

void ev_init(ev_t *e);

void ev_set_state(ev_t *e, ev_channel_t ch, uint8_t state);

void ev_raise(ev_t *e, ev_oneshot_t ev, uint32_t value);

/* Obstacle level with hysteresis. Distances in cm, EV_DIST_NONE if none; azimuth in degrees, positive
 * to the right. In ground phases only the real-obstacle distance is used (DIA-04). */
void ev_obstacle(ev_t *e, const ev_cfg_t *cfg, phase_t ph, uint16_t exported_cm, int16_t exported_az,
                 uint16_t real_cm, int16_t real_az);

/* Next STATUSTEXT allowed at now_us; false if none is due or the rate limit holds it back.
 * text gets up to 50 characters plus a terminator. */
bool ev_next_statustext(ev_t *e, uint32_t now_us, uint8_t *severity, char text[EV_TEXT_LEN + 1u]);

void ev_log(ev_t *e, uint32_t t_us, ev_log_kind_t kind, uint8_t a, uint16_t b);
bool ev_take_log(ev_t *e, ev_log_t *out);

#endif /* CORE_EVENTS_H */
