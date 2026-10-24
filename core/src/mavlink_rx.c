#include "core/mavlink_rx.h"

#include <string.h>

#define PI_F 3.14159265f

/* Signed difference a - b of wrapping module times. */
static int32_t tdiff(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b);
}

/* Age of t at now; a stamp slightly in the future counts as 0. */
static uint32_t age_us(uint32_t now_us, uint32_t t_us)
{
    const int32_t d = tdiff(now_us, t_us);
    return (d < 0) ? 0u : (uint32_t)d;
}

/* REQ: ICD-03 */
void mavrx_init(mavrx_t *r, uint8_t own_compid, uint8_t cfg_sysid)
{
    memset(r, 0, sizeof *r);
    r->own_compid = own_compid;
    r->cfg_sysid = cfg_sysid;
}

static bool from_fc(const mavrx_t *r, const mavlink_message_t *m)
{
    return r->fc_known && m->sysid == r->fc_sysid && m->compid == MAVRX_FC_COMPID;
}

/* REQ: ICD-01 (section 6, HEARTBEAT FC: mode and armed), FLT-08 (first HEARTBEAT) */
static void on_heartbeat(mavrx_t *r, const mavlink_message_t *m, uint32_t now_us)
{
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(m, &hb);
    /* The FC: autopilot component, not a GCS or another onboard computer forwarded by it. */
    if (m->compid != MAVRX_FC_COMPID || hb.autopilot == MAV_AUTOPILOT_INVALID || hb.type == MAV_TYPE_GCS) {
        r->stats.foreign++;
        return;
    }
    if (!r->fc_known) {
        if (r->cfg_sysid != 0u && m->sysid != r->cfg_sysid) {
            r->stats.foreign++;
            return;
        }
        r->fc_known = true;
        r->fc_sysid = m->sysid;
    } else if (m->sysid != r->fc_sysid) {
        r->stats.foreign++;
        return;
    }
    if (!r->hb_seen) {
        r->first_hb_evt = true; /* cleared by the consumer */
    }
    r->hb_seen = true;
    r->t_hb_us = now_us;
    r->hb_count++;
    r->armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0u;
    r->custom_mode = hb.custom_mode;
}

/* REQ: MAP-14 (ATTITUDE ring buffer >= 300 ms) */
static void on_attitude(mavrx_t *r, const mavlink_message_t *m, uint32_t now_us)
{
    mavlink_attitude_t a;
    mavlink_msg_attitude_decode(m, &a);
    if (r->att_count > 0u &&
        (uint32_t)(now_us - r->att[(r->att_head + MAVRX_ATT_LEN - 1u) % MAVRX_ATT_LEN].t_us) < MAVRX_ATT_MIN_STEP_US) {
        return; /* faster than 100 Hz: decimated so the ring keeps >= 300 ms */
    }
    mavrx_att_t *s = &r->att[r->att_head];
    s->t_us = now_us;
    s->roll_rad = a.roll;
    s->pitch_rad = a.pitch;
    r->att_head = (uint8_t)((r->att_head + 1u) % MAVRX_ATT_LEN);
    if (r->att_count < MAVRX_ATT_LEN) {
        r->att_count++;
    }
}

/* REQ: MAP-15 (DISTANCE_SENSOR orientation 25 only), MAP-15c (below minimum vs fault) */
static void on_distance_sensor(mavrx_t *r, const mavlink_message_t *m, uint32_t now_us)
{
    mavlink_distance_sensor_t d;
    mavlink_msg_distance_sensor_decode(m, &d);
    if (d.orientation != MAVRX_RF_ORIENTATION) {
        return; /* other orientations are ignored */
    }
    r->rf_seen = true;
    r->t_rf_us = now_us;
    /* signal_quality 1 = invalid signal (0 = unknown): a fault like a reading above the maximum. */
    const bool usable = d.max_distance > 0u && d.signal_quality != 1u;
    const bool low = usable && d.current_distance <= d.min_distance;
    r->rf_in_range = usable && !low && d.current_distance <= d.max_distance;
    if (r->rf_in_range) {
        r->rf_valid_seen = true;
        r->t_rf_valid_us = now_us;
        r->rf_range_m = (float)d.current_distance / 100.0f;
    }
    if (low) {
        if (!r->rf_low_run) {
            r->rf_low_run = true;
            r->t_rf_low_since_us = now_us;
        }
        r->rf_low_seen = true;
        r->t_rf_low_us = now_us;
    } else {
        r->rf_low_run = false;
    }
}

/* REQ: ICD-04 */
static void refuse(mavrx_t *r, const mavlink_message_t *m, uint16_t command, uint8_t target_sys,
                   uint8_t target_comp)
{
    const bool to_me = target_comp == r->own_compid &&
                       (target_sys == 0u || !r->fc_known || target_sys == r->fc_sysid);
    if (!to_me) {
        r->stats.foreign++; /* broadcast or for others: ignored, never executed */
        return;
    }
    r->stats.cmds_refused++;
    if (r->nack_count >= MAVRX_NACK_LEN) {
        r->stats.nack_dropped++;
        return;
    }
    mavrx_nack_t *n = &r->nack[r->nack_count++];
    n->command = command;
    n->sysid = m->sysid;
    n->compid = m->compid;
}

/* REQ: ICD-01 (incoming table), ICD-04, FLT-03 (COMMAND_ACK logged only) */
static void dispatch(mavrx_t *r, const mavlink_message_t *m, uint32_t now_us)
{
    switch (m->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
        on_heartbeat(r, m, now_us);
        return;
    case MAVLINK_MSG_ID_COMMAND_LONG: {
        mavlink_command_long_t c;
        mavlink_msg_command_long_decode(m, &c);
        refuse(r, m, c.command, c.target_system, c.target_component);
        return;
    }
    case MAVLINK_MSG_ID_COMMAND_INT: {
        mavlink_command_int_t c;
        mavlink_msg_command_int_decode(m, &c);
        refuse(r, m, c.command, c.target_system, c.target_component);
        return;
    }
    default:
        break;
    }
    if (!from_fc(r, m)) {
        r->stats.foreign++;
        return;
    }
    switch (m->msgid) {
    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
        r->ess_seen = true;
        r->t_ess_us = now_us;
        r->landed_state = mavlink_msg_extended_sys_state_get_landed_state(m);
        break;
    case MAVLINK_MSG_ID_ATTITUDE:
        on_attitude(r, m, now_us);
        break;
    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        on_distance_sensor(r, m, now_us);
        break;
    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t a;
        mavlink_msg_command_ack_decode(m, &a);
        if ((a.target_system != 0u && a.target_system != r->fc_sysid) ||
            (a.target_component != 0u && a.target_component != r->own_compid)) {
            r->stats.foreign++; /* an ACK for the GCS or another component */
            break;
        }
        r->ack_evt = true;
        r->ack_command = a.command;
        r->ack_result = a.result;
        r->t_ack_us = now_us;
        break;
    }
    default:
        break; /* not used */
    }
}

/* REQ: ICD-03 (garbage and truncation tolerated, bad frames dropped, CRC errors counted) */
void mavrx_feed(mavrx_t *r, const uint8_t *data, uint32_t len, uint32_t now_us)
{
    for (uint32_t k = 0; k < len; k++) {
        const uint8_t res = mavlink_frame_char_buffer(&r->rx_buf, &r->rx_status, data[k], &r->msg, &r->msg_status);
        r->stats.bytes++;
        /* The parser keeps an 8-bit counter; follow its increments. */
        r->stats.parse_errors += (uint8_t)(r->rx_status.parse_error - r->last_parse_error);
        r->last_parse_error = r->rx_status.parse_error;
        if (res == MAVLINK_FRAMING_OK) {
            r->stats.msgs++;
            dispatch(r, &r->msg, now_us);
        } else if (res == MAVLINK_FRAMING_BAD_CRC || res == MAVLINK_FRAMING_BAD_SIGNATURE) {
            r->stats.crc_errors++;
        } else {
            /* incomplete */
        }
    }
}

/* REQ: MAP-16 (HEARTBEAT > 3 s: FC_LINK_LOST) */
bool mavrx_hb_valid(const mavrx_t *r, uint32_t now_us)
{
    return r->hb_seen && age_us(now_us, r->t_hb_us) <= MAVRX_HB_TIMEOUT_US;
}

/* REQ: MAP-16 (EXTENDED_SYS_STATE > 3 s: PHASE_UNKNOWN) */
bool mavrx_ess_valid(const mavrx_t *r, uint32_t now_us)
{
    return r->ess_seen && age_us(now_us, r->t_ess_us) <= MAVRX_ESS_TIMEOUT_US;
}

static const mavrx_att_t *att_newest(const mavrx_t *r, uint8_t back)
{
    return &r->att[(r->att_head + MAVRX_ATT_LEN - 1u - back) % MAVRX_ATT_LEN];
}

/* REQ: MAP-14 (ATTITUDE > 200 ms: orientation invalid) */
bool mavrx_att_valid(const mavrx_t *r, uint32_t now_us)
{
    return r->att_count > 0u && age_us(now_us, att_newest(r, 0u)->t_us) <= MAVRX_ATT_TIMEOUT_US;
}

static float wrap_pi(float a)
{
    if (a > PI_F) {
        return a - 2.0f * PI_F;
    }
    if (a < -PI_F) {
        return a + 2.0f * PI_F;
    }
    return a;
}

/* REQ: MAP-14, 5.1 (ATTITUDE interpolated at the capture time) */
bool mavrx_att_at(const mavrx_t *r, uint32_t t_us, uint32_t now_us, float *pitch_rad, float *roll_rad)
{
    if (!mavrx_att_valid(r, now_us)) {
        return false;
    }
    const mavrx_att_t *b = att_newest(r, 0u);
    if (tdiff(t_us, b->t_us) >= 0) {
        if ((uint32_t)tdiff(t_us, b->t_us) > MAVRX_ATT_TIMEOUT_US) {
            return false;
        }
        *pitch_rad = b->pitch_rad;
        *roll_rad = b->roll_rad;
        return true;
    }
    for (uint8_t k = 1u; k < r->att_count; k++) {
        const mavrx_att_t *a = att_newest(r, k);
        if (tdiff(t_us, a->t_us) >= 0) {
            const uint32_t span = (uint32_t)tdiff(b->t_us, a->t_us);
            if (span > MAVRX_ATT_TIMEOUT_US) {
                return false; /* gap in the stream around t_us */
            }
            const float w = (span == 0u) ? 0.0f : (float)(uint32_t)tdiff(t_us, a->t_us) / (float)span;
            *pitch_rad = a->pitch_rad + w * (b->pitch_rad - a->pitch_rad);
            *roll_rad = wrap_pi(a->roll_rad + w * wrap_pi(b->roll_rad - a->roll_rad));
            return true;
        }
        b = a;
    }
    return false; /* older than the buffer */
}

/* REQ: MAP-15, MAP-15b (no valid DISTANCE_SENSOR for > 500 ms: height invalid) */
bool mavrx_rf_valid(const mavrx_t *r, uint32_t now_us, float *range_m)
{
    return mavrx_rf_state(r, now_us, range_m) == MAVRX_RF_OK;
}

/* REQ: MAP-15b, MAP-15c */
mavrx_rf_state_t mavrx_rf_state(const mavrx_t *r, uint32_t now_us, float *range_m)
{
    const bool ok = r->rf_valid_seen && age_us(now_us, r->t_rf_valid_us) <= MAVRX_RF_TIMEOUT_US;
    const bool low = r->rf_low_seen && age_us(now_us, r->t_rf_low_us) <= MAVRX_RF_TIMEOUT_US;
    if (ok && (!low || tdiff(r->t_rf_valid_us, r->t_rf_low_us) > 0)) {
        *range_m = r->rf_range_m;
        return MAVRX_RF_OK;
    }
    return low ? MAVRX_RF_LOW : MAVRX_RF_FAULT;
}

/* REQ: MAP-15c (RF_LOW_MAX_TAKEOFF_S) */
bool mavrx_rf_low_since(const mavrx_t *r, uint32_t now_us, uint32_t *since_us)
{
    if (!r->rf_low_run || age_us(now_us, r->t_rf_low_us) > MAVRX_RF_TIMEOUT_US) {
        return false;
    }
    *since_us = r->t_rf_low_since_us;
    return true;
}

/* REQ: ICD-04 */
bool mavrx_take_nack(mavrx_t *r, mavrx_nack_t *out)
{
    if (r->nack_count == 0u) {
        return false;
    }
    *out = r->nack[0];
    r->nack_count--;
    for (uint8_t k = 0; k < r->nack_count; k++) {
        r->nack[k] = r->nack[k + 1u];
    }
    return true;
}
