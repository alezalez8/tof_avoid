#ifndef CORE_PHASE_H
#define CORE_PHASE_H

#include <stdbool.h>
#include <stdint.h>

#include "core/params.h"

/* Flight phase automaton (MAP-16) driven only by FC data: armed (HEARTBEAT), landed_state
 * (EXTENDED_SYS_STATE) and h_tof (MAP-15). Export policy for GROUND_SUSPECT points by phase (MAP-17,
 * MAP-07, MAP-07a) and phases in which the hold command is allowed (FLT-03, FLT-05). */

typedef enum {
    PHASE_UNKNOWN = 0, /* no FC HEARTBEAT or EXTENDED_SYS_STATE for > 3 s (also at start) */
    PHASE_GROUND,
    PHASE_ARMED_GROUND,
    PHASE_TAKEOFF,
    PHASE_FLIGHT,
    PHASE_LANDING,
    PHASE_GROUND_RECOVER, /* one step after a flight: cells, holds and fault episodes are reset */
    PHASE_COUNT
} phase_t;

/* MAV_LANDED_STATE values (MAVLink common). */
#define PHASE_LS_UNDEFINED 0u
#define PHASE_LS_ON_GROUND 1u
#define PHASE_LS_IN_AIR 2u
#define PHASE_LS_TAKEOFF 3u
#define PHASE_LS_LANDING 4u

#define PHASE_FLIGHT_CONFIRM_US 1000000u /* h_tof >= PHASE_H_FLIGHT_M for 1 s */

typedef struct {
    float h_flight_m; /* PHASE_H_FLIGHT_M */
    float h_hyst_m;   /* PHASE_H_HYST_M */
    uint8_t ground_policy;
} phase_cfg_t;

typedef struct {
    bool hb_valid;  /* FC HEARTBEAT received within 3 s */
    bool armed;
    bool ess_valid; /* EXTENDED_SYS_STATE received within 3 s */
    uint8_t landed_state;
    float h_m; /* h_tof by MAP-15 for the current phase (rangefinder, or the ground parameter) */
} phase_input_t;

typedef struct {
    phase_t phase;
    phase_t last_known;   /* phase before PHASE_UNKNOWN */
    bool airborne;        /* TAKEOFF / FLIGHT / LANDING since the last GROUND_RECOVER */
    bool h_timer_on;      /* TAKEOFF -> FLIGHT confirmation running */
    uint32_t t_h_since_us;
    uint32_t t_enter_us;  /* time the current phase was entered */
} phase_fsm_t;

/* phase_update result flags. */
#define PHASE_EVT_NONE 0u
#define PHASE_EVT_CHANGED 0x01u
#define PHASE_EVT_RESET 0x02u /* GROUND_RECOVER entered: reset cells, holds, fault episodes */

void phase_cfg_from_params(phase_cfg_t *cfg, const params_t *p);

void phase_init(phase_fsm_t *f, uint32_t now_us);

uint8_t phase_update(phase_fsm_t *f, const phase_cfg_t *cfg, const phase_input_t *in, uint32_t now_us);

/* MAP-17: true if GROUND_SUSPECT points are exported as obstacles (KEEP), false if they are sent as
 * 65535. GROUND_POLICY applies only in FLIGHT; DROP needs a build with TOF_ALLOW_GROUND_DROP and a
 * valid height (MAP-07, MAP-07a). Real obstacles are exported in every phase. */
bool phase_export_ground(phase_t ph, uint8_t ground_policy, bool height_valid);

/* FLT-03, FLT-05: phases in which the hold command may be sent. PHASE_UNKNOWN only if the FC mode is
 * known. */
bool phase_hold_allowed(phase_t ph, bool mode_known);

/* Ground phases: GROUND_SUSPECT never raises obstacle warnings (DIA-04). */
bool phase_is_ground(phase_t ph);

/* Three-letter name for LCD and logs. */
const char *phase_name(phase_t ph);

#endif /* CORE_PHASE_H */
