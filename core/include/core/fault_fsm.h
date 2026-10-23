#ifndef CORE_FAULT_FSM_H
#define CORE_FAULT_FSM_H

#include <stdbool.h>
#include <stdint.h>

#include "core/fusion.h"
#include "core/params.h"
#include "core/phase.h"
#include "core/sensor_health.h"

/* Reaction to the module's own faults (FLT-03...FLT-06, FLT-08): an attempt to put the FC into BRAKE
 * or LOITER. An additional mechanism, not a guaranteed failsafe (section 16). The automaton decides;
 * the scheduler encodes DO_SET_MODE (mavtx_hold_mode). Fault classes are PARAM_FAULT_BIT_* (FLT-01),
 * detected by the scheduler. */

#define FAULT_RETRY_US 500000u /* repeat every 500 ms */
#define FAULT_MAX_SENDS 6u     /* FLT-03a: first transmission + up to 5 repeats */

typedef struct {
    uint8_t action;       /* FAULT_ACTION: PARAM_FAULT_NONE / BRAKE / LOITER */
    uint8_t trigger_mask; /* FAULT_TRIGGER_MASK */
    bool on_reboot;       /* FAULT_ON_REBOOT */
} fault_cfg_t;

/* FC state as seen by the module (mavlink_rx). */
typedef struct {
    bool mode_known;      /* FC HEARTBEAT within 3 s */
    bool armed;
    uint32_t custom_mode;
    uint32_t hb_count;    /* FC HEARTBEATs received so far */
    bool first_hb;        /* the first FC HEARTBEAT after start arrived since the last update */
    phase_t phase;
} fault_fc_t;

typedef enum {
    FAULT_SERIES_NONE = 0, /* episode without a command series yet */
    FAULT_SERIES_SENDING,  /* commands sent, waiting for the mode in HEARTBEAT */
    FAULT_SERIES_SUCCEEDED,
    FAULT_SERIES_REJECTED, /* all transmissions without a mode change (FLT-06) */
    FAULT_SERIES_ABORTED   /* the mode left AUTO/GUIDED/RTL by other means, or disarm */
} fault_series_t;

typedef struct {
    uint8_t active;    /* fault classes present at the last update */
    bool episode;      /* triggered classes present, or reboot latch (FLT-04) */
    bool reboot_latch; /* FLT-08: first FC HEARTBEAT armed in AUTO/GUIDED/RTL */
    fault_series_t series;
    uint8_t sends;
    uint32_t t_last_send_us;
    uint32_t hb_at_first_send;
    uint32_t target_mode;
    bool was_auto;     /* FC was in AUTO/GUIDED/RTL at the last update */
    uint32_t episodes_total;
    uint32_t sends_total;
} fault_fsm_t;

/* fault_update events (bit flags), turned into STATUSTEXT and log records by events. */
#define FAULT_EVT_NONE 0x00u
#define FAULT_EVT_EPISODE_START 0x01u
#define FAULT_EVT_EPISODE_END 0x02u
#define FAULT_EVT_HOLD_SENT 0x04u     /* first transmission of a series */
#define FAULT_EVT_HOLD_OK 0x08u       /* mode changed, seen in HEARTBEAT */
#define FAULT_EVT_HOLD_REJECTED 0x10u /* FLT-06 */
#define FAULT_EVT_REBOOTED_IN_FLIGHT 0x20u /* FLT-08 */
#define FAULT_EVT_PILOT_RESUMED 0x40u /* FLT-04: AUTO again under the same fault, warning only */

typedef struct {
    bool send;            /* send DO_SET_MODE now */
    uint32_t mode;        /* TOF_COPTER_MODE_BRAKE or _LOITER */
    uint8_t confirmation; /* 0 first, then 1, 2, ... */
    uint8_t events;
} fault_out_t;

void fault_cfg_from_params(fault_cfg_t *cfg, const params_t *p);

void fault_init(fault_fsm_t *f);

/* GROUND_RECOVER (5.4): fault episodes are reset. */
void fault_reset_episode(fault_fsm_t *f);

/* active: PARAM_FAULT_BIT_* present now. Call every main loop pass. */
void fault_update(fault_fsm_t *f, const fault_cfg_t *cfg, uint8_t active, const fault_fc_t *fc, uint32_t now_us,
                  fault_out_t *out);

/* The command of the last fault_update could not be queued: it does not count as a transmission and
 * is retried on the next pass. */
void fault_send_failed(fault_fsm_t *f);

/* AUTO, GUIDED or RTL. */
bool fault_is_auto_mode(uint32_t custom_mode);

/* FLT-10 "virtual wall" (FAULT_WALL, default 0): the sectors of every FAILED sensor are set to
 * min_distance. Call only when FAULT_WALL = 1. */
void fault_wall_apply(const fusion_t *fu, const sensor_health_t health[TOF_NUM_SENSORS], uint16_t min_cm,
                      uint16_t distances[TOF_NUM_SECTORS]);

#endif /* CORE_FAULT_FSM_H */
