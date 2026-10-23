#ifndef CORE_SCHEDULER_H
#define CORE_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

#include "core/events.h"
#include "core/fault_fsm.h"
#include "core/ground.h"
#include "core/mavlink_rx.h"
#include "core/mavlink_tx.h"
#include "core/phase.h"
#include "core/tof_core.h"

/* Core pipeline of the cooperative main loop (SW-05): sensor frame -> map -> OBSTACLE_DISTANCE, with
 * FC telemetry, flight phase, fault reaction and events. The app feeds UART bytes and frames, calls
 * sched_step() every pass and moves the TX queue to the UART DMA. Time is always an argument.
 *
 * Latency stamps of one map (MAP-12, section 9):
 *   t1 - ready time of the frame that defines the map (oldest frame used, or the trigger frame in
 *        STAGGERED, MAP-09a);
 *   t2 - that frame handed to the core (read complete);
 *   t3 - fused map ready;
 *   t4 - OBSTACLE_DISTANCE taken by the UART DMA;
 *   t5 - UART transmission of it complete. */

#define SCHED_TXQ_LEN 2048u
#define SCHED_HB_PERIOD_US 1000000u
#define SCHED_NV_PERIOD_US 1000000u
#define SCHED_REQ_PERIOD_US 5000000u    /* SET_MESSAGE_INTERVAL repeated while a stream is missing */
#define SCHED_OVERFLOW_HOLD_US 1000000u /* internal fault class kept after a TX queue overflow */

typedef enum {
    SCHED_STREAM_ATTITUDE = 0,
    SCHED_STREAM_DISTANCE_SENSOR,
    SCHED_STREAM_EXT_SYS_STATE,
    SCHED_STREAM_COUNT
} sched_stream_t;

/* One map for the debug log (DIA-06, MAP-12). */
typedef struct {
    uint32_t seq;
    uint32_t t1_us;
    uint32_t t2_us;
    uint32_t t3_us;
    uint32_t t4_us; /* 0 until sent */
    uint32_t t5_us; /* 0 until sent */
    bool sent;      /* false: LOST, rate limit or no MAVLink identity */
    uint8_t phase;
    uint8_t map_state;
    uint8_t view;   /* fusion_view_t exported */
    bool att_valid; /* ATTITUDE at t1 */
    float pitch_rad;
    float roll_rad;
    uint32_t fc_mode;
    bool fc_armed;
    uint8_t faults; /* PARAM_FAULT_BIT_* active */
    uint16_t distances[TOF_NUM_SECTORS];
} sched_map_log_t;

typedef struct {
    uint32_t od_sent;
    uint32_t od_skipped_rate;
    uint32_t od_skipped_lost;
    uint32_t tx_overflows;
    uint32_t map_logs_dropped;
} sched_stats_t;

typedef struct {
    params_t params;
    tof_core_t core;
    mavrx_t rx;
    mavtx_t tx;
    phase_cfg_t phase_cfg;
    phase_fsm_t phase;
    fault_cfg_t fault_cfg;
    fault_fsm_t fault;
    ev_cfg_t ev_cfg;
    ev_t ev;
    ground_height_in_t hin; /* RF_XYZ_M, TOF_HEIGHT_GROUND_M; rangefinder filled per use */

    uint8_t cfg_sysid;
    uint16_t min_cm;
    uint16_t max_cm;
    uint32_t stale_us;     /* 2 x VALID_DATA_MAX_AGE_MS (FLT-01) */
    uint32_t od_period_us; /* 1 / MAV_DISTANCE_RATE_HZ */
    bool wall;             /* FAULT_WALL */
    bool named_values;     /* NAMED_VALUE_ENABLE */

    uint32_t time_hi;      /* wraps of now_us, for the 64-bit time_usec */
    uint32_t last_now_us;
    uint32_t t_last_map_us;
    bool have_map;
    bool ready_raised;
    bool link_seen;
    bool phase_seen;
    bool internal_fault;
    uint32_t t_overflow_us;
    bool overflow_recent;
    uint8_t faults;

    bool hb_sent;
    uint32_t t_hb_us;
    uint32_t t_nv_us;
    bool req_sent[SCHED_STREAM_COUNT];
    uint32_t t_req_us[SCHED_STREAM_COUNT];
    uint32_t od_tokens_us; /* token bucket of the OBSTACLE_DISTANCE rate */
    uint32_t t_tokens_us;

    uint8_t txq[SCHED_TXQ_LEN];
    uint32_t q_w; /* bytes queued (absolute) */
    uint32_t q_r; /* bytes taken by the DMA */
    uint32_t q_d; /* bytes reported sent */
    bool od_in_queue;
    uint32_t od_start;
    uint32_t od_end;

    sched_map_log_t log_cur; /* waiting for t4/t5 */
    bool log_cur_busy;
    sched_map_log_t log_out;
    bool log_out_ready;

    sched_stats_t stats;
} sched_t;

/* p must pass params_validate(). params_defaults: both slots were invalid (SW-07). */
void sched_init(sched_t *s, const params_t *p, bool params_defaults, uint32_t now_us);

void sched_uart_rx(sched_t *s, const uint8_t *data, uint32_t len, uint32_t now_us);

/* A frame from the driver; ATTITUDE is interpolated at its ready time for the ground check. */
tof_push_status_t sched_push_frame(sched_t *s, const tof_frame_t *frame, uint32_t now_us);

/* One pass of the main loop. */
void sched_step(sched_t *s, uint32_t now_us);

/* Moves up to max queued bytes to dst for the UART DMA; returns the count. */
uint32_t sched_tx_take(sched_t *s, uint8_t *dst, uint32_t max, uint32_t now_us);

/* The DMA finished every byte taken so far. */
void sched_tx_done(sched_t *s, uint32_t now_us);

/* Internal error reported by the app (stack, buffers): FLT-01 class INTERNAL while set. */
void sched_set_internal_fault(sched_t *s, bool on);

bool sched_take_map_log(sched_t *s, sched_map_log_t *out);
bool sched_take_event_log(sched_t *s, ev_log_t *out);

#endif /* CORE_SCHEDULER_H */
