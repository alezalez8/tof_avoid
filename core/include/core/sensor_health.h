#ifndef CORE_SENSOR_HEALTH_H
#define CORE_SENSOR_HEALTH_H

#include <stdbool.h>
#include <stdint.h>

#include "core/params.h"

/* Per-sensor supervisor (SNS-11) and health (MAP-10). The core decides, the driver acts:
 * the app polls sh_take_reinit_request() and reports the result with sh_on_reinit_done(). */

#define SH_ERRORS_TO_REINIT 3u      /* consecutive driver errors */
#define SH_PERIODS_TO_REINIT 3u     /* no frame for longer than 3 periods */
#define SH_REINIT_FAILS_TO_FAILED 5u
#define SH_FAILED_RETRY_US 10000000u /* retry every 10 s in FAILED */
#define SH_REINIT_TIMEOUT_US 1000000u /* a reinit not reported within 1 s counts as failed */

typedef enum {
    SENSOR_OK = 0,
    SENSOR_DEGRADED,
    SENSOR_FAILED
} sensor_health_t;

typedef enum {
    SH_RUNNING = 0, /* ranging, frames expected */
    SH_REINIT,      /* reinit requested or in progress (only this sensor) */
    SH_FAILED       /* 5 failed reinits in a row; retry every 10 s */
} sh_sup_state_t;

typedef struct {
    uint32_t period_us;        /* frame period from SENSOR_FREQ_HZ */
    float valid_frac_min;      /* VALID_FRAC_MIN */
    uint32_t degraded_time_us; /* DEGRADED_TIME_MS */
} sh_cfg_t;

typedef struct {
    sh_sup_state_t sup;
    sensor_health_t health;
    uint8_t consec_errors;
    uint8_t consec_reinit_fails;
    bool reinit_request;     /* pending, taken by the app */
    bool reinit_unconfirmed; /* reinit reported OK, no frame yet */
    uint32_t t_last_frame_us;
    uint32_t t_reinit_us;    /* start of the current reinit / last retry in FAILED */
    bool low_active;         /* valid fraction low, ambient high, or no data (MAP-10, MAP-10a) */
    uint32_t t_low_since_us;
    uint32_t reinit_total;   /* statistics */
} sh_t;

void sh_cfg_from_params(sh_cfg_t *cfg, const params_t *p);

/* Starts supervision at now_us (ranging just started: the first frame is due within 3 periods). */
void sh_init(sh_t *s, uint32_t now_us);

/* A frame arrived; valid_frac is the share of valid zones among zones with a target and
 * ambient_high is "median ambient above AMBIENT_DEGRADED" (MAP-10a). Either condition held for
 * longer than DEGRADED_TIME_MS gives DEGRADED. */
void sh_on_frame(sh_t *s, const sh_cfg_t *cfg, float valid_frac, bool ambient_high, uint32_t now_us);

/* The driver reported an error (SPI timeout, bad data) for this sensor. */
void sh_on_error(sh_t *s, uint32_t now_us);

/* True once per reinit request; the app then reinitialises this sensor only. */
bool sh_take_reinit_request(sh_t *s);

/* Result of the reinit started after sh_take_reinit_request(). */
void sh_on_reinit_done(sh_t *s, bool ok, uint32_t now_us);

/* Timers: missing frames, reinit timeout, FAILED retry, health. Call every main loop pass. */
void sh_update(sh_t *s, const sh_cfg_t *cfg, uint32_t now_us);

#endif /* CORE_SENSOR_HEALTH_H */
