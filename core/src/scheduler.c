#include "core/scheduler.h"

#include <string.h>

/* REQ: SW-05, SW-07 (event on defaults), SNS-12a (event at start) */
void sched_init(sched_t *s, const params_t *p, bool params_defaults, uint32_t now_us)
{
    memset(s, 0, sizeof *s);
    s->params = *p;
    tof_core_init(&s->core, p, now_us);
    s->cfg_sysid = (uint8_t)params_get_u32(p, PARAM_MAV_SYSID);
    const uint8_t compid = (uint8_t)params_get_u32(p, PARAM_MAV_COMPID);
    mavrx_init(&s->rx, compid, s->cfg_sysid);
    mavtx_init(&s->tx, s->cfg_sysid, compid);
    phase_cfg_from_params(&s->phase_cfg, p);
    phase_init(&s->phase, now_us);
    fault_cfg_from_params(&s->fault_cfg, p);
    fault_init(&s->fault);
    ev_cfg_from_params(&s->ev_cfg, p);
    ev_init(&s->ev);
    s->hin.rf_xyz.x = params_get_f32(p, PARAM_RF_X_M);
    s->hin.rf_xyz.y = params_get_f32(p, PARAM_RF_Y_M);
    s->hin.rf_xyz.z = params_get_f32(p, PARAM_RF_Z_M);
    s->hin.h_opt_ground_m = params_get_f32(p, PARAM_TOF_HEIGHT_GROUND_M);
    s->min_cm = (uint16_t)params_get_u32(p, PARAM_MIN_DIST_CM);
    s->max_cm = (uint16_t)params_get_u32(p, PARAM_MAX_DIST_CM);
    s->stale_us = 2u * params_get_u32(p, PARAM_VALID_DATA_MAX_AGE_MS) * 1000u;
    s->od_period_us = 1000000u / params_get_u32(p, PARAM_MAV_DISTANCE_RATE_HZ);
    s->od_tokens_us = s->od_period_us;
    s->wall = params_get_u32(p, PARAM_FAULT_WALL) != 0u;
    s->named_values = params_get_u32(p, PARAM_NAMED_VALUE_ENABLE) != 0u;
    s->last_now_us = now_us;
    s->t_last_map_us = now_us;
    s->t_tokens_us = now_us;
    if (params_defaults) {
        ev_raise(&s->ev, EV_ONE_PARAMS_DEFAULT, 0u);
    }
    if (s->core.orient_not_confirmed_event) {
        ev_raise(&s->ev, EV_ONE_ORIENT, 0u);
        s->core.orient_not_confirmed_event = false;
    }
}

/* REQ: ICD-03, ICD-05 (bytes from the UART DMA) */
void sched_uart_rx(sched_t *s, const uint8_t *data, uint32_t len, uint32_t now_us)
{
    mavrx_feed(&s->rx, data, len, now_us);
}

/* Rangefinder state for the height of MAP-15. */
static void fill_rf(sched_t *s, uint32_t now_us)
{
    s->hin.rf_valid = mavrx_rf_valid(&s->rx, now_us, &s->hin.rf_range_m);
}

/* REQ: MAP-14, 5.1 (capture time = mid-exposure, DEV-12 p. 6) */
uint32_t sched_capture_time_us(const sched_t *s, uint32_t t_ready_us)
{
    const uint32_t off_ms = params_get_u32(&s->params, PARAM_EXPOSURE_OFFSET_MS);
    const uint32_t off_us = (off_ms > 0u) ? off_ms * 1000u : 500000u / params_get_u32(&s->params, PARAM_SENSOR_FREQ_HZ);
    return t_ready_us - off_us;
}

/* REQ: MAP-14, MAP-14a, MAP-14b, MAP-15, MAP-15b (ground context of a frame) */
tof_push_status_t sched_push_frame(sched_t *s, const tof_frame_t *frame, uint32_t now_us)
{
    ground_ctx_t g;
    memset(&g, 0, sizeof g);
    g.tol_m = params_get_f32(&s->params, PARAM_GROUND_TOL_M);
    g.cal_tol_deg = params_get_f32(&s->params, PARAM_CAL_TOL_DEG);
    g.cal_tol_el_deg = params_get_f32(&s->params, PARAM_CAL_TOL_EL_DEG);
    if (frame->sensor_id < TOF_NUM_SENSORS &&
        mavrx_att_at(&s->rx, sched_capture_time_us(s, frame->t_ready_us), now_us, &g.pitch_rad, &g.roll_rad)) {
        fill_rf(s, now_us);
        const ground_h_src_t src = ground_body_height(s->phase.phase, &s->hin, &s->core.fusion.mount[frame->sensor_id],
                                                      g.pitch_rad, g.roll_rad, &g.h_body_m);
        g.valid = src != GROUND_H_NONE; /* MAP-14: no orientation or no height - no flag */
    }
    return tof_core_push_frame(&s->core, frame, &g, now_us);
}

/* ---- TX queue ---- */

/* REQ: ICD-05 (no blocking: a packet that does not fit is dropped and counted) */
static bool enqueue(sched_t *s, const uint8_t *buf, uint16_t len, uint32_t now_us)
{
    if (len == 0u) {
        return false;
    }
    if (SCHED_TXQ_LEN - (s->q_w - s->q_r) < len) {
        s->stats.tx_overflows++;
        s->overflow_recent = true;
        s->t_overflow_us = now_us;
        return false;
    }
    for (uint16_t k = 0; k < len; k++) {
        s->txq[(s->q_w + k) % SCHED_TXQ_LEN] = buf[k];
    }
    s->q_w += len;
    return true;
}

static bool can_send(const sched_t *s)
{
    return s->tx.sysid != 0u;
}

/* REQ: MAP-12 (t4) */
uint32_t sched_tx_take(sched_t *s, uint8_t *dst, uint32_t max, uint32_t now_us)
{
    const uint32_t n = ((s->q_w - s->q_r) < max) ? (s->q_w - s->q_r) : max;
    for (uint32_t k = 0; k < n; k++) {
        dst[k] = s->txq[(s->q_r + k) % SCHED_TXQ_LEN];
    }
    if (s->od_in_queue && s->log_cur_busy && s->log_cur.t4_us == 0u && (int32_t)(s->od_start - s->q_r) >= 0 &&
        (int32_t)(s->od_start - (s->q_r + n)) < 0) {
        s->log_cur.t4_us = now_us;
    }
    s->q_r += n;
    return n;
}

static void log_publish(sched_t *s)
{
    if (s->log_out_ready) {
        s->stats.map_logs_dropped++;
    }
    s->log_out = s->log_cur;
    s->log_out_ready = true;
    s->log_cur_busy = false;
    s->od_in_queue = false;
}

/* REQ: MAP-12 (t5) */
void sched_tx_done(sched_t *s, uint32_t now_us)
{
    s->q_d = s->q_r;
    if (s->od_in_queue && s->log_cur_busy && (int32_t)(s->q_d - s->od_end) >= 0) {
        s->log_cur.t5_us = now_us;
        log_publish(s);
    }
}

/* REQ: FLT-01 */
void sched_set_internal_fault(sched_t *s, bool on)
{
    s->internal_fault = on;
}

/* REQ: DIA-06, MAP-12 */
bool sched_take_map_log(sched_t *s, sched_map_log_t *out)
{
    if (!s->log_out_ready) {
        return false;
    }
    *out = s->log_out;
    s->log_out_ready = false;
    return true;
}

/* REQ: DIA-06 */
bool sched_take_event_log(sched_t *s, ev_log_t *out)
{
    return ev_take_log(&s->ev, out);
}

/* ---- main loop pass ---- */

static void health_array(const sched_t *s, sensor_health_t h[TOF_NUM_SENSORS])
{
    for (uint8_t k = 0; k < TOF_NUM_SENSORS; k++) {
        h[k] = tof_core_sensor_health(&s->core, k);
    }
}

/* REQ: ICD-04 (UNSUPPORTED replies), FLT-03 (COMMAND_ACK logged) */
static void handle_rx(sched_t *s, uint32_t now_us)
{
    if (s->tx.sysid == 0u && s->rx.fc_known) {
        s->tx.sysid = s->rx.fc_sysid; /* section 6: sysid of the FC */
    }
    if (s->rx.ack_evt) {
        s->rx.ack_evt = false;
        ev_log(&s->ev, now_us, EV_LOG_COMMAND_ACK, s->rx.ack_result, s->rx.ack_command);
    }
    mavrx_nack_t n;
    while (can_send(s) && mavrx_take_nack(&s->rx, &n)) {
        uint8_t buf[MAVTX_BUF_LEN];
        (void)enqueue(s, buf, mavtx_command_ack(&s->tx, buf, n.command, MAV_RESULT_UNSUPPORTED, n.sysid, n.compid),
                      now_us);
        ev_log(&s->ev, now_us, EV_LOG_NACK, 0u, n.command);
    }
}

/* Body height for the phase automaton: rangefinder only (MAP-15b); false if not valid. */
static bool phase_height(sched_t *s, uint32_t now_us, float *h)
{
    float pitch = 0.0f;
    float roll = 0.0f;
    if (!mavrx_att_at(&s->rx, now_us, now_us, &pitch, &roll)) {
        pitch = 0.0f;
        roll = 0.0f;
    }
    return ground_body_height(PHASE_FLIGHT, &s->hin, &s->core.fusion.mount[TOF_SENSOR_C], pitch, roll, h) ==
           GROUND_H_RANGEFINDER;
}

/* REQ: MAP-16, MAP-17 (GROUND_RECOVER reset) */
static void update_phase(sched_t *s, uint32_t now_us)
{
    phase_input_t in;
    in.hb_valid = mavrx_hb_valid(&s->rx, now_us);
    in.armed = s->rx.armed;
    in.ess_valid = mavrx_ess_valid(&s->rx, now_us);
    in.landed_state = s->rx.landed_state;
    in.h_m = 0.0f;
    in.h_valid = phase_height(s, now_us, &in.h_m);
    const phase_t prev = s->phase.phase;
    const uint8_t evt = phase_update(&s->phase, &s->phase_cfg, &in, now_us);
    if ((evt & PHASE_EVT_RESET) != 0u) {
        fusion_reset_cells(&s->core.fusion);
        fault_reset_episode(&s->fault);
    }
    if ((evt & PHASE_EVT_CHANGED) != 0u) {
        ev_log(&s->ev, now_us, EV_LOG_PHASE, (uint8_t)s->phase.phase, (uint16_t)prev);
    }
    s->phase_seen = s->phase_seen || s->phase.phase != PHASE_UNKNOWN;
}

/* Height valid for GROUND_POLICY = DROP in FLIGHT (MAP-07): rangefinder and ATTITUDE. */
static bool drop_height_valid(const sched_t *s, uint32_t now_us)
{
    return s->hin.rf_valid && mavrx_att_valid(&s->rx, now_us);
}

/* REQ: FLT-01 (fault classes, detection <= 300 ms) */
static uint8_t detect_faults(sched_t *s, uint32_t now_us)
{
    sensor_health_t h[TOF_NUM_SENSORS];
    health_array(s, h);
    uint8_t f = 0u;
    for (uint8_t k = 0; k < TOF_NUM_SENSORS; k++) {
        f |= (h[k] == SENSOR_FAILED) ? PARAM_FAULT_BIT_SENSOR_FAILED : 0u;
        f |= (h[k] == SENSOR_DEGRADED) ? PARAM_FAULT_BIT_SENSOR_DEGRADED : 0u;
    }
    if ((uint32_t)(now_us - s->t_last_map_us) > s->stale_us) {
        f |= PARAM_FAULT_BIT_MAP_STALE; /* no fused map for longer than 2 x VALID_DATA_MAX_AGE_MS */
    }
    if (tof_core_map_state(&s->core) == MAP_LOST) {
        f |= PARAM_FAULT_BIT_MAP_LOST;
    }
    if (params_get_u32(&s->params, PARAM_GROUND_POLICY) == PARAM_GROUND_DROP && s->phase.phase == PHASE_FLIGHT &&
        !drop_height_valid(s, now_us)) {
        f |= PARAM_FAULT_BIT_ATT_ALT_LOST;
    }
    if (s->overflow_recent && (uint32_t)(now_us - s->t_overflow_us) > SCHED_OVERFLOW_HOLD_US) {
        s->overflow_recent = false;
    }
    if (s->internal_fault || s->overflow_recent) {
        f |= PARAM_FAULT_BIT_INTERNAL;
    }
    return f;
}

/* REQ: FLT-03, FLT-04, FLT-06, FLT-08 */
static void update_faults(sched_t *s, uint32_t now_us)
{
    const uint8_t before = s->faults;
    s->faults = detect_faults(s, now_us);
    fault_fc_t fc;
    fc.mode_known = mavrx_hb_valid(&s->rx, now_us);
    fc.armed = s->rx.armed;
    fc.custom_mode = s->rx.custom_mode;
    fc.hb_count = s->rx.hb_count;
    fc.first_hb = s->rx.first_hb_evt;
    fc.phase = s->phase.phase;
    s->rx.first_hb_evt = false;
    fault_out_t out;
    fault_update(&s->fault, &s->fault_cfg, s->faults, &fc, now_us, &out);
    bool queued = false;
    if (out.send) {
        uint8_t buf[MAVTX_BUF_LEN];
        queued = can_send(s) &&
                 enqueue(s, buf, mavtx_hold_mode(&s->tx, buf, s->rx.fc_sysid, out.mode, out.confirmation), now_us);
        if (queued) {
            ev_log(&s->ev, now_us, EV_LOG_HOLD_COMMAND, out.confirmation, (uint16_t)out.mode);
        } else {
            fault_send_failed(&s->fault); /* FLT-03: retried on the next pass, not counted */
        }
    }
    if ((out.events & FAULT_EVT_HOLD_SENT) != 0u && queued) {
        ev_raise(&s->ev, EV_ONE_HOLD_SENT, out.mode);
    }
    if ((out.events & FAULT_EVT_HOLD_REJECTED) != 0u) {
        ev_raise(&s->ev, EV_ONE_HOLD_REJECTED, 0u);
    }
    if ((out.events & FAULT_EVT_REBOOTED_IN_FLIGHT) != 0u) {
        ev_raise(&s->ev, EV_ONE_REBOOTED, 0u);
    }
    if ((out.events & FAULT_EVT_PILOT_RESUMED) != 0u) {
        ev_raise(&s->ev, EV_ONE_PILOT_RESUMED, 0u);
    }
    if ((out.events & (FAULT_EVT_EPISODE_START | FAULT_EVT_EPISODE_END)) != 0u || before != s->faults) {
        ev_log(&s->ev, now_us, EV_LOG_FAULT, s->faults, (out.events & FAULT_EVT_EPISODE_START) != 0u ? 1u : 0u);
    }
}

/* Nearest OCCUPIED sector of a view, for DIA-04 warnings. */
static void nearest(const sched_t *s, fusion_view_t view, uint32_t now_us, uint16_t *d_cm, int16_t *az_deg)
{
    *d_cm = EV_DIST_NONE;
    *az_deg = 0;
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        if (fusion_export_state_view(&s->core.fusion, view, k, now_us) != EXPORT_OCCUPIED) {
            continue;
        }
        const cell_t *c = (view == FUSION_VIEW_REAL) ? &s->core.fusion.cell_real[k] : &s->core.fusion.cell[k];
        if (c->dist_cm < *d_cm) {
            *d_cm = c->dist_cm;
            *az_deg = (int16_t)((k <= 36u) ? (int32_t)k * 5 : (int32_t)k * 5 - 360);
        }
    }
}

/* REQ: MAP-17, MAP-07, ICD-02, FLT-02, FLT-10, MAP-12 (log record), MAP-13 (hold expiry logged) */
static void on_new_map(sched_t *s, uint32_t now_us)
{
    fusion_t *fu = &s->core.fusion;
    for (uint8_t v = 0; v < FUSION_VIEW_COUNT; v++) {
        for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
            if (fu->hold_expired[v][k]) {
                ev_log(&s->ev, now_us, EV_LOG_HOLD_EXPIRED, v, k);
                fu->hold_expired[v][k] = false;
            }
        }
    }
    const uint8_t policy = (uint8_t)params_get_u32(&s->params, PARAM_GROUND_POLICY);
    const fusion_view_t view = phase_export_ground(s->phase.phase, policy, drop_height_valid(s, now_us))
                                   ? FUSION_VIEW_ALL
                                   : FUSION_VIEW_REAL;
    if (s->log_cur_busy) {
        log_publish(s); /* the previous map is still in the UART: published without t5 */
    }
    sched_map_log_t *lg = &s->log_cur;
    memset(lg, 0, sizeof *lg);
    lg->seq = fu->map_seq;
    lg->t1_us = fu->t1_us;
    lg->t2_us = s->core.t_push_us[fu->t1_sensor];
    lg->t3_us = fu->t_map_us;
    lg->phase = (uint8_t)s->phase.phase;
    lg->map_state = (uint8_t)tof_core_map_state(&s->core);
    lg->view = (uint8_t)view;
    lg->att_valid =
        mavrx_att_at(&s->rx, sched_capture_time_us(s, fu->t1_us), now_us, &lg->pitch_rad, &lg->roll_rad);
    lg->fc_mode = s->rx.custom_mode;
    lg->fc_armed = s->rx.armed;
    lg->faults = s->faults;
    fusion_export_view(fu, view, now_us, lg->distances);
    if (s->wall) {
        sensor_health_t h[TOF_NUM_SENSORS];
        health_array(s, h);
        fault_wall_apply(fu, h, s->min_cm, lg->distances);
    }

    /* Token bucket: MAV_DISTANCE_RATE_HZ on average, at most two periods of credit. */
    const uint32_t dt = now_us - s->t_tokens_us;
    s->t_tokens_us = now_us;
    s->od_tokens_us = (dt >= 2u * s->od_period_us - s->od_tokens_us) ? 2u * s->od_period_us : s->od_tokens_us + dt;
    bool sent = false;
    if (tof_core_map_state(&s->core) == MAP_LOST) {
        s->stats.od_skipped_lost++; /* FLT-02: FC goes NoData after 500 ms */
    } else if (s->od_tokens_us < s->od_period_us) {
        s->stats.od_skipped_rate++;
    } else if (can_send(s)) {
        uint8_t buf[MAVTX_BUF_LEN];
        const uint64_t t64 = ((uint64_t)s->time_hi << 32) | now_us;
        const uint16_t n = mavtx_obstacle_distance(&s->tx, buf, t64, lg->distances, s->min_cm, s->max_cm);
        const uint32_t start = s->q_w;
        if (enqueue(s, buf, n, now_us)) {
            s->od_tokens_us -= s->od_period_us;
            s->od_in_queue = true;
            s->od_start = start;
            s->od_end = start + n;
            s->stats.od_sent++;
            sent = true;
        }
    } else {
        /* no MAVLink identity yet */
    }
    lg->sent = sent;
    s->log_cur_busy = true;
    if (!sent) {
        log_publish(s);
    }

    /* DIA-04: warnings from what the FC receives; in ground phases from real obstacles only. */
    uint16_t d_exp = EV_DIST_NONE;
    uint16_t d_real = EV_DIST_NONE;
    int16_t az_exp = 0;
    int16_t az_real = 0;
    nearest(s, view, now_us, &d_exp, &az_exp);
    nearest(s, FUSION_VIEW_REAL, now_us, &d_real, &az_real);
    ev_obstacle(&s->ev, &s->ev_cfg, s->phase.phase, d_exp, az_exp, d_real, az_real);
    if (!s->ready_raised && tof_core_map_state(&s->core) != MAP_LOST) {
        s->ready_raised = true;
        ev_raise(&s->ev, EV_ONE_READY, 0u);
    }
}

/* REQ: DIA-03 (state channels) */
static void update_event_states(sched_t *s, uint32_t now_us)
{
    sensor_health_t h[TOF_NUM_SENSORS];
    health_array(s, h);
    for (uint8_t k = 0; k < TOF_NUM_SENSORS; k++) {
        ev_set_state(&s->ev, (ev_channel_t)(EV_CH_SENSOR_L + k), (h[k] == SENSOR_FAILED) ? 1u : 0u);
    }
    ev_set_state(&s->ev, EV_CH_STALE, (s->have_map && (s->faults & PARAM_FAULT_BIT_MAP_STALE) != 0u) ? 1u : 0u);
    ev_set_state(&s->ev, EV_CH_MAP_LOST, ((s->faults & PARAM_FAULT_BIT_MAP_LOST) != 0u) ? 1u : 0u);
    const bool link = mavrx_hb_valid(&s->rx, now_us);
    s->link_seen = s->link_seen || link;
    ev_set_state(&s->ev, EV_CH_FC_LINK, (s->link_seen && !link) ? 1u : 0u);
    ev_set_state(&s->ev, EV_CH_PHASE, (s->phase_seen && s->phase.phase == PHASE_UNKNOWN) ? 1u : 0u);
}

static uint8_t percent(float v)
{
    return (uint8_t)(v * 100.0f + 0.5f);
}

/* REQ: ICD-01 (HEARTBEAT 1 Hz, STATUSTEXT, SET_MESSAGE_INTERVAL, NAMED_VALUE_INT), FLT-02 */
static void periodic_tx(sched_t *s, uint32_t now_us)
{
    if (!can_send(s)) {
        return; /* section 6: no identity before the first FC HEARTBEAT (MAV_SYSID = 0) */
    }
    uint8_t buf[MAVTX_BUF_LEN];
    sensor_health_t h[TOF_NUM_SENSORS];
    health_array(s, h);
    if (!s->hb_sent || (uint32_t)(now_us - s->t_hb_us) >= SCHED_HB_PERIOD_US) {
        bool critical = tof_core_map_state(&s->core) == MAP_LOST;
        for (uint8_t k = 0; k < TOF_NUM_SENSORS; k++) {
            critical = critical || h[k] == SENSOR_FAILED;
        }
        (void)enqueue(s, buf, mavtx_heartbeat(&s->tx, buf, critical), now_us);
        s->hb_sent = true;
        s->t_hb_us = now_us;
    }
    uint8_t sev = 0u;
    char text[EV_TEXT_LEN + 1u];
    while (ev_next_statustext(&s->ev, now_us, &sev, text)) {
        (void)enqueue(s, buf, mavtx_statustext(&s->tx, buf, sev, text), now_us);
    }
    if (s->rx.fc_known) {
        /* At start and every 5 s while a stream is not arriving. */
        static const uint32_t k_msg[SCHED_STREAM_COUNT] = {MAVLINK_MSG_ID_ATTITUDE, MAVLINK_MSG_ID_DISTANCE_SENSOR,
                                                           MAVLINK_MSG_ID_EXTENDED_SYS_STATE};
        static const int32_t k_int[SCHED_STREAM_COUNT] = {MAVTX_ATT_INTERVAL_US, MAVTX_RF_INTERVAL_US,
                                                          MAVTX_ESS_INTERVAL_US};
        const bool present[SCHED_STREAM_COUNT] = {
            mavrx_att_valid(&s->rx, now_us),
            s->rx.rf_seen && (uint32_t)(now_us - s->rx.t_rf_us) <= MAVRX_RF_TIMEOUT_US,
            mavrx_ess_valid(&s->rx, now_us)};
        for (uint8_t k = 0; k < SCHED_STREAM_COUNT; k++) {
            if (!present[k] && (!s->req_sent[k] || (uint32_t)(now_us - s->t_req_us[k]) >= SCHED_REQ_PERIOD_US)) {
                (void)enqueue(s, buf, mavtx_set_message_interval(&s->tx, buf, s->rx.fc_sysid, k_msg[k], k_int[k]),
                              now_us);
                s->req_sent[k] = true;
                s->t_req_us[k] = now_us;
            }
        }
    }
    if (s->named_values && (uint32_t)(now_us - s->t_nv_us) >= SCHED_NV_PERIOD_US) {
        /* TOF_HEALTH: health L | C << 2 | R << 4; TOF_VALID: valid zones % L | C << 8 | R << 16;
         * TOF_MAP: map state (0 OK, 1 DEGRADED, 2 LOST). */
        const uint32_t ms = now_us / 1000u;
        int32_t health = 0;
        int32_t valid = 0;
        for (uint8_t k = 0; k < TOF_NUM_SENSORS; k++) {
            health |= (int32_t)h[k] << (2u * k);
            valid |= (int32_t)percent(s->core.fusion.frame[k].valid_frac) << (8u * k);
        }
        (void)enqueue(s, buf, mavtx_named_value_int(&s->tx, buf, ms, "TOF_HEALTH", health), now_us);
        (void)enqueue(s, buf, mavtx_named_value_int(&s->tx, buf, ms, "TOF_VALID", valid), now_us);
        (void)enqueue(s, buf, mavtx_named_value_int(&s->tx, buf, ms, "TOF_MAP", (int32_t)tof_core_map_state(&s->core)),
                      now_us);
        s->t_nv_us = now_us;
    }
}

/* REQ: SW-05, MAP-12, ICD-02 (one OBSTACLE_DISTANCE per fused map) */
void sched_step(sched_t *s, uint32_t now_us)
{
    if (now_us < s->last_now_us) {
        s->time_hi++; /* now_us wrapped (~71 min) */
    }
    s->last_now_us = now_us;
    fill_rf(s, now_us);
    handle_rx(s, now_us);
    const bool produced = tof_core_update(&s->core, now_us);
    if (produced) {
        s->have_map = true;
        s->t_last_map_us = now_us;
    }
    update_phase(s, now_us);
    update_faults(s, now_us);
    if (produced) {
        on_new_map(s, now_us);
    }
    update_event_states(s, now_us);
    periodic_tx(s, now_us);
}
