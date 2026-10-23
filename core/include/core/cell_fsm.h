#ifndef CORE_CELL_FSM_H
#define CORE_CELL_FSM_H

#include <stdbool.h>
#include <stdint.h>

#include "core/params.h"

/* Hold and release of an obstacle in one map cell (MAP-13, table 5.3).
 * Invalid data never removes an obstacle. STALE is not a state here: it is decided at export from
 * the cell data time (MAP-09, 5.2). */

typedef enum {
    CELL_UNKNOWN = 0,
    CELL_OCCUPIED,
    CELL_CLEAR_MEASURED
} cell_state_t;

/* Candidate from one fused frame: OCC - at least one zone gives a valid obstacle (the nearest is
 * taken); CLR - all zones valid and no target up to max_distance; INV - everything else. */
typedef enum {
    CELL_CAND_INV = 0,
    CELL_CAND_OCC,
    CELL_CAND_CLR
} cell_cand_kind_t;

typedef struct {
    cell_cand_kind_t kind;
    uint16_t dist_cm;    /* OCC only */
    uint8_t src_sensor;  /* OCC only: sensor and raw zone of the nearest target */
    uint8_t src_zone;
    bool angle_ok;       /* OCC only: zone extent hits the sector without CAL_TOL_DEG (DEV-06) */
} cell_input_t;

typedef struct {
    uint8_t clear_confirm_frames;
    uint8_t dist_relax_frames;
    uint32_t hold_max_us;
    bool clear_enabled; /* CLEAR_MEASURED allowed (AMBIENT_CLEAR_MAX > 0, DEV-02) */
} cell_cfg_t;

typedef struct {
    cell_state_t state;
    uint16_t dist_cm;      /* held distance, OCCUPIED only */
    uint32_t t_valid_us;   /* time of the last valid obstacle measurement */
    uint8_t src_sensor;    /* source of the held distance */
    uint8_t src_zone;
    bool angle_ok;         /* angular reliability of the held distance */
    bool ground_suspect;   /* GROUND_SUSPECT flag, set by ground classification (MAP-14, later) */
    uint8_t clr_count;     /* consecutive CLR candidates */
    uint8_t relax_count;   /* consecutive farther OCC candidates */
    uint16_t relax_min_cm; /* nearest distance seen in the current relax run */
    uint8_t relax_src_sensor;
    uint8_t relax_src_zone;
    bool relax_angle_ok;
} cell_t;

/* Events returned by cell_fsm_step (bit flags). */
#define CELL_EVT_NONE 0u
#define CELL_EVT_OCCUPIED 0x01u     /* entered OCCUPIED from another state */
#define CELL_EVT_HOLD_EXPIRED 0x02u /* OCCUPIED -> UNKNOWN after HOLD_MAX_MS of INV (logged) */
#define CELL_EVT_RELEASED 0x04u     /* OCCUPIED -> CLEAR_MEASURED / UNKNOWN after confirmed CLR */

void cell_cfg_from_params(cell_cfg_t *cfg, const params_t *p);

void cell_init(cell_t *c);

/* Applies one candidate at time now_us; returns CELL_EVT_* flags. */
uint8_t cell_fsm_step(cell_t *c, const cell_cfg_t *cfg, const cell_input_t *in, uint32_t now_us);

#endif /* CORE_CELL_FSM_H */
