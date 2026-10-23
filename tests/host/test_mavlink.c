#include <math.h>
#include <string.h>

#include "core/mavlink_rx.h"
#include "core/mavlink_tx.h"
#include "unity.h"

#define MS 1000u
#define FC_SYS 7u

static mavtx_t g_tx;
static mavrx_t g_rx;
static uint8_t g_buf[MAVTX_BUF_LEN];
static mavlink_status_t g_fc_status; /* sequence of the simulated FC */

void setUp(void)
{
    mavtx_init(&g_tx, FC_SYS, 196u);
    mavrx_init(&g_rx, 196u, 0u);
    memset(&g_fc_status, 0, sizeof g_fc_status);
}

void tearDown(void) {}

/* Decodes one packet of buf; fails the test if it is not exactly one good frame. */
static void decode(const uint8_t *buf, uint16_t len, mavlink_message_t *out)
{
    mavlink_message_t work;
    mavlink_status_t st;
    mavlink_status_t rst;
    memset(&work, 0, sizeof work);
    memset(&st, 0, sizeof st);
    uint8_t res = MAVLINK_FRAMING_INCOMPLETE;
    for (uint16_t k = 0; k < len; k++) {
        res = mavlink_frame_char_buffer(&work, &st, buf[k], out, &rst);
        if (k + 1u < len) {
            TEST_ASSERT_EQUAL_UINT8(MAVLINK_FRAMING_INCOMPLETE, res);
        }
    }
    TEST_ASSERT_EQUAL_UINT8(MAVLINK_FRAMING_OK, res);
    /* Plain MAVLink 2 as ArduPilot 4.7.2 expects: no incompatibility flags (32-bit ids, signing). */
    TEST_ASSERT_EQUAL_HEX8(MAVLINK_STX, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0u, buf[2]);
}

/* ---- packets of a simulated FC ---- */

static uint16_t fc_heartbeat(uint8_t *buf, uint8_t sys, uint8_t comp, uint8_t type, uint8_t autopilot,
                             bool armed, uint32_t mode)
{
    mavlink_message_t m;
    (void)mavlink_msg_heartbeat_pack_status(sys, comp, &g_fc_status, &m, type, autopilot,
                                            (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                                      (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0u)),
                                            mode, MAV_STATE_ACTIVE);
    return mavlink_msg_to_send_buffer(buf, &m);
}

static void feed_hb(bool armed, uint32_t mode, uint32_t now)
{
    uint8_t b[MAVTX_BUF_LEN];
    const uint16_t n = fc_heartbeat(b, FC_SYS, 1u, MAV_TYPE_HEXAROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, armed, mode);
    mavrx_feed(&g_rx, b, n, now);
}

static void feed_att(float pitch, float roll, uint32_t now)
{
    uint8_t b[MAVTX_BUF_LEN];
    mavlink_message_t m;
    (void)mavlink_msg_attitude_pack_status(FC_SYS, 1u, &g_fc_status, &m, now / 1000u, roll, pitch, 0.0f, 0.0f,
                                           0.0f, 0.0f);
    mavrx_feed(&g_rx, b, mavlink_msg_to_send_buffer(b, &m), now);
}

static void feed_ess(uint8_t ls, uint32_t now)
{
    uint8_t b[MAVTX_BUF_LEN];
    mavlink_message_t m;
    (void)mavlink_msg_extended_sys_state_pack_status(FC_SYS, 1u, &g_fc_status, &m, MAV_VTOL_STATE_UNDEFINED, ls);
    mavrx_feed(&g_rx, b, mavlink_msg_to_send_buffer(b, &m), now);
}

static void feed_rf(uint8_t sys, uint16_t cm, uint8_t orient, uint8_t quality, uint32_t now)
{
    uint8_t b[MAVTX_BUF_LEN];
    mavlink_message_t m;
    const float q[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    (void)mavlink_msg_distance_sensor_pack_status(sys, 1u, &g_fc_status, &m, now / 1000u, 10u, 700u, cm,
                                                  MAV_DISTANCE_SENSOR_LASER, 0u, orient, 0u, 0.0f, 0.0f, q,
                                                  quality);
    mavrx_feed(&g_rx, b, mavlink_msg_to_send_buffer(b, &m), now);
}

static uint16_t cmd_long(uint8_t *b, uint8_t sys, uint8_t comp, uint8_t tsys, uint8_t tcomp, uint16_t cmd)
{
    mavlink_message_t m;
    (void)mavlink_msg_command_long_pack_status(sys, comp, &g_fc_status, &m, tsys, tcomp, cmd, 0u, 1.0f, 2.0f,
                                               0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    return mavlink_msg_to_send_buffer(b, &m);
}

/* ---- ICD-01: encoder ---- */

/* ICD-01: OBSTACLE_DISTANCE exactly by the field table of section 6. */
void test_ICD01_obstacle_distance_fields(void)
{
    uint16_t d[TOF_NUM_SECTORS];
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        d[k] = (uint16_t)(65535u - k);
    }
    d[0] = 120u;
    d[71] = 301u;
    const uint16_t n = mavtx_obstacle_distance(&g_tx, g_buf, 0x123456789ull, d, 4u, 300u);
    TEST_ASSERT_TRUE(n > 0u);
    TEST_ASSERT_EQUAL_HEX8(MAVLINK_STX, g_buf[0]); /* MAVLink 2 */
    mavlink_message_t m;
    decode(g_buf, n, &m);
    TEST_ASSERT_EQUAL_UINT32(MAVLINK_MSG_ID_OBSTACLE_DISTANCE, m.msgid);
    TEST_ASSERT_EQUAL_UINT32(FC_SYS, m.sysid);
    TEST_ASSERT_EQUAL_UINT8(196u, m.compid);
    mavlink_obstacle_distance_t od;
    mavlink_msg_obstacle_distance_decode(&m, &od);
    TEST_ASSERT_EQUAL_UINT64(0x123456789ull, od.time_usec);
    TEST_ASSERT_EQUAL_UINT8(MAV_DISTANCE_SENSOR_LASER, od.sensor_type);
    TEST_ASSERT_EQUAL_UINT8(0u, od.sensor_type);
    TEST_ASSERT_EQUAL_UINT8(MAV_FRAME_BODY_FRD, od.frame);
    TEST_ASSERT_EQUAL_UINT8(12u, od.frame);
    TEST_ASSERT_EQUAL_UINT8(0u, od.increment);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, od.increment_f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, od.angle_offset);
    TEST_ASSERT_EQUAL_UINT16(4u, od.min_distance);
    TEST_ASSERT_EQUAL_UINT16(300u, od.max_distance);
    TEST_ASSERT_EQUAL_UINT16_ARRAY(d, od.distances, TOF_NUM_SECTORS);
}

/* ICD-01: HEARTBEAT type / autopilot / status; sequence numbers advance per packet. */
void test_ICD01_heartbeat_fields_and_sequence(void)
{
    mavlink_message_t m;
    mavlink_heartbeat_t hb;
    decode(g_buf, mavtx_heartbeat(&g_tx, g_buf, false), &m);
    mavlink_msg_heartbeat_decode(&m, &hb);
    TEST_ASSERT_EQUAL_UINT8(MAV_TYPE_ONBOARD_CONTROLLER, hb.type);
    TEST_ASSERT_EQUAL_UINT8(MAV_AUTOPILOT_INVALID, hb.autopilot);
    TEST_ASSERT_EQUAL_UINT8(MAV_STATE_ACTIVE, hb.system_status);
    const uint8_t seq0 = m.seq;
    decode(g_buf, mavtx_heartbeat(&g_tx, g_buf, true), &m);
    mavlink_msg_heartbeat_decode(&m, &hb);
    TEST_ASSERT_EQUAL_UINT8(MAV_STATE_CRITICAL, hb.system_status);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(seq0 + 1u), m.seq);
}

/* ICD-01, DIA-03: STATUSTEXT severity and text, cut to 50 characters. */
void test_ICD01_statustext_cut_to_50(void)
{
    mavlink_message_t m;
    char text[MAVTX_TEXT_LEN + 1];
    const char *longer = "TOF 0123456789012345678901234567890123456789012345678901234567";
    decode(g_buf, mavtx_statustext(&g_tx, g_buf, MAV_SEVERITY_CRITICAL, longer), &m);
    TEST_ASSERT_EQUAL_UINT8(MAV_SEVERITY_CRITICAL, mavlink_msg_statustext_get_severity(&m));
    memset(text, 0, sizeof text);
    (void)mavlink_msg_statustext_get_text(&m, text);
    TEST_ASSERT_EQUAL_UINT32(50u, strlen(text));
    TEST_ASSERT_EQUAL_MEMORY(longer, text, 50u);
    decode(g_buf, mavtx_statustext(&g_tx, g_buf, MAV_SEVERITY_INFO, "TOF READY"), &m);
    memset(text, 0, sizeof text);
    (void)mavlink_msg_statustext_get_text(&m, text);
    TEST_ASSERT_EQUAL_STRING("TOF READY", text);
}

/* ICD-01: SET_MESSAGE_INTERVAL to the FC autopilot. */
void test_ICD01_set_message_interval(void)
{
    mavlink_message_t m;
    mavlink_command_long_t c;
    decode(g_buf, mavtx_set_message_interval(&g_tx, g_buf, FC_SYS, MAVLINK_MSG_ID_ATTITUDE, MAVTX_ATT_INTERVAL_US), &m);
    TEST_ASSERT_EQUAL_UINT32(MAVLINK_MSG_ID_COMMAND_LONG, m.msgid);
    mavlink_msg_command_long_decode(&m, &c);
    TEST_ASSERT_EQUAL_UINT16(MAV_CMD_SET_MESSAGE_INTERVAL, c.command);
    TEST_ASSERT_EQUAL_UINT16(511u, c.command);
    TEST_ASSERT_EQUAL_UINT8(FC_SYS, c.target_system);
    TEST_ASSERT_EQUAL_UINT8(1u, c.target_component);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, c.param1);
    TEST_ASSERT_EQUAL_FLOAT(20000.0f, c.param2);
}

/* FLT-03, FLT-05: DO_SET_MODE only to BRAKE (17) or LOITER (5), param1 = 1. */
void test_FLT05_hold_mode_only_brake_or_loiter(void)
{
    mavlink_message_t m;
    mavlink_command_long_t c;
    decode(g_buf, mavtx_hold_mode(&g_tx, g_buf, FC_SYS, TOF_COPTER_MODE_BRAKE, 0u), &m);
    mavlink_msg_command_long_decode(&m, &c);
    TEST_ASSERT_EQUAL_UINT16(MAV_CMD_DO_SET_MODE, c.command);
    TEST_ASSERT_EQUAL_UINT16(176u, c.command);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, c.param1);
    TEST_ASSERT_EQUAL_FLOAT(17.0f, c.param2);
    decode(g_buf, mavtx_hold_mode(&g_tx, g_buf, FC_SYS, TOF_COPTER_MODE_LOITER, 3u), &m);
    mavlink_msg_command_long_decode(&m, &c);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, c.param2);
    TEST_ASSERT_EQUAL_UINT8(3u, c.confirmation);
    TEST_ASSERT_EQUAL_UINT16(0u, mavtx_hold_mode(&g_tx, g_buf, FC_SYS, TOF_COPTER_MODE_AUTO, 0u));
    TEST_ASSERT_EQUAL_UINT16(0u, mavtx_hold_mode(&g_tx, g_buf, FC_SYS, 0u, 0u));
}

/* FLT-03: mode numbers match the ardupilotmega dialect. */
void test_FLT03_copter_mode_numbers(void)
{
    TEST_ASSERT_EQUAL_UINT32(COPTER_MODE_AUTO, TOF_COPTER_MODE_AUTO);
    TEST_ASSERT_EQUAL_UINT32(COPTER_MODE_GUIDED, TOF_COPTER_MODE_GUIDED);
    TEST_ASSERT_EQUAL_UINT32(COPTER_MODE_RTL, TOF_COPTER_MODE_RTL);
    TEST_ASSERT_EQUAL_UINT32(COPTER_MODE_LOITER, TOF_COPTER_MODE_LOITER);
    TEST_ASSERT_EQUAL_UINT32(COPTER_MODE_BRAKE, TOF_COPTER_MODE_BRAKE);
}

/* ICD-01: NAMED_VALUE_INT and COMMAND_ACK. */
void test_ICD01_named_value_and_ack(void)
{
    mavlink_message_t m;
    char name[11];
    decode(g_buf, mavtx_named_value_int(&g_tx, g_buf, 1234u, "TOF_HEALTH_LONG", -5), &m);
    memset(name, 0, sizeof name);
    (void)mavlink_msg_named_value_int_get_name(&m, name);
    TEST_ASSERT_EQUAL_STRING("TOF_HEALTH", name);
    TEST_ASSERT_EQUAL_INT32(-5, mavlink_msg_named_value_int_get_value(&m));
    TEST_ASSERT_EQUAL_UINT32(1234u, mavlink_msg_named_value_int_get_time_boot_ms(&m));
    decode(g_buf, mavtx_command_ack(&g_tx, g_buf, 400u, MAV_RESULT_UNSUPPORTED, 255u, 190u), &m);
    TEST_ASSERT_EQUAL_UINT16(400u, mavlink_msg_command_ack_get_command(&m));
    TEST_ASSERT_EQUAL_UINT8(MAV_RESULT_UNSUPPORTED, mavlink_msg_command_ack_get_result(&m));
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&m, &ack);
    TEST_ASSERT_EQUAL_UINT8(255u, ack.target_system);
    TEST_ASSERT_EQUAL_UINT8(190u, ack.target_component);
}

/* ---- incoming ---- */

/* Section 6: the first FC HEARTBEAT gives the sysid, armed and mode; GCS and other components are
 * ignored. */
void test_ICD01_rx_fc_heartbeat(void)
{
    uint8_t b[MAVTX_BUF_LEN];
    mavrx_feed(&g_rx, b, fc_heartbeat(b, 255u, 190u, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, false, 0u), 0u);
    mavrx_feed(&g_rx, b, fc_heartbeat(b, FC_SYS, 1u, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, false, 0u), 0u);
    mavrx_feed(&g_rx, b, fc_heartbeat(b, FC_SYS, 2u, MAV_TYPE_HEXAROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, true, 3u), 0u);
    TEST_ASSERT_FALSE(g_rx.fc_known);
    TEST_ASSERT_EQUAL_UINT32(3u, g_rx.stats.foreign);
    feed_hb(true, TOF_COPTER_MODE_AUTO, 10u * MS);
    TEST_ASSERT_TRUE(g_rx.fc_known);
    TEST_ASSERT_EQUAL_UINT8(FC_SYS, g_rx.fc_sysid);
    TEST_ASSERT_TRUE(g_rx.armed);
    TEST_ASSERT_EQUAL_UINT32(TOF_COPTER_MODE_AUTO, g_rx.custom_mode);
    TEST_ASSERT_TRUE(g_rx.first_hb_evt);
    g_rx.first_hb_evt = false;
    feed_hb(false, TOF_COPTER_MODE_LOITER, 20u * MS);
    TEST_ASSERT_FALSE(g_rx.first_hb_evt);
    TEST_ASSERT_FALSE(g_rx.armed);
    TEST_ASSERT_EQUAL_UINT32(2u, g_rx.hb_count);
    /* Another autopilot system later is ignored. */
    mavrx_feed(&g_rx, b, fc_heartbeat(b, 9u, 1u, MAV_TYPE_HEXAROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, true, 6u), 30u * MS);
    TEST_ASSERT_EQUAL_UINT32(TOF_COPTER_MODE_LOITER, g_rx.custom_mode);
}

/* Section 6: MAV_SYSID set - only that FC is accepted. */
void test_ICD01_rx_configured_sysid(void)
{
    uint8_t b[MAVTX_BUF_LEN];
    mavrx_init(&g_rx, 196u, 9u);
    feed_hb(true, TOF_COPTER_MODE_AUTO, 0u);
    TEST_ASSERT_FALSE(g_rx.fc_known);
    mavrx_feed(&g_rx, b, fc_heartbeat(b, 9u, 1u, MAV_TYPE_HEXAROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, false, 0u), 0u);
    TEST_ASSERT_TRUE(g_rx.fc_known);
    TEST_ASSERT_EQUAL_UINT8(9u, g_rx.fc_sysid);
}

/* Section 6: HEARTBEAT and EXTENDED_SYS_STATE are valid for 3 s. */
void test_MAP16_rx_link_timeouts_3s(void)
{
    TEST_ASSERT_FALSE(mavrx_hb_valid(&g_rx, 0u));
    TEST_ASSERT_FALSE(mavrx_ess_valid(&g_rx, 0u));
    feed_hb(false, 0u, 1000u * MS);
    feed_ess(MAV_LANDED_STATE_ON_GROUND, 1000u * MS);
    TEST_ASSERT_EQUAL_UINT8(MAV_LANDED_STATE_ON_GROUND, g_rx.landed_state);
    TEST_ASSERT_TRUE(mavrx_hb_valid(&g_rx, 4000u * MS));
    TEST_ASSERT_TRUE(mavrx_ess_valid(&g_rx, 4000u * MS));
    TEST_ASSERT_FALSE(mavrx_hb_valid(&g_rx, 4000u * MS + 1u));
    TEST_ASSERT_FALSE(mavrx_ess_valid(&g_rx, 4000u * MS + 1u));
}

/* Data from a system other than the FC is ignored (before and after the FC is known). */
void test_ICD01_rx_foreign_data_ignored(void)
{
    feed_ess(MAV_LANDED_STATE_IN_AIR, 0u); /* FC not known yet */
    TEST_ASSERT_FALSE(g_rx.ess_seen);
    feed_hb(false, 0u, 0u);
    feed_rf(42u, 150u, MAVRX_RF_ORIENTATION, 100u, 0u);
    TEST_ASSERT_FALSE(g_rx.rf_seen);
}

/* MAP-14: ATTITUDE interpolated at the capture time from a ring buffer >= 300 ms. */
void test_MAP14_attitude_interpolation(void)
{
    feed_hb(false, 0u, 0u);
    for (uint32_t k = 0; k <= 50u; k++) { /* 50 Hz for 1 s, pitch ramps 0.01 rad per sample */
        feed_att(0.01f * (float)k, -0.02f * (float)k, 1000u * MS + k * 20u * MS);
    }
    const uint32_t now = 2000u * MS;
    float p = 0.0f;
    float r = 0.0f;
    /* Midpoint between samples 45 and 46. */
    TEST_ASSERT_TRUE(mavrx_att_at(&g_rx, 1000u * MS + 45u * 20u * MS + 10u * MS, now, &p, &r));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.455f, p);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.91f, r);
    /* 300 ms back is inside the buffer. */
    TEST_ASSERT_TRUE(mavrx_att_at(&g_rx, now - 300u * MS, now, &p, &r));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.35f, p);
    /* Older than the 32-sample buffer (640 ms at 50 Hz). */
    TEST_ASSERT_FALSE(mavrx_att_at(&g_rx, now - 700u * MS, now, &p, &r));
    /* Newer than the newest sample: held up to 200 ms. */
    TEST_ASSERT_TRUE(mavrx_att_at(&g_rx, now + 100u * MS, now + 100u * MS, &p, &r));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, p);
    TEST_ASSERT_FALSE(mavrx_att_at(&g_rx, now + 201u * MS, now + 150u * MS, &p, &r));
}

/* Section 6: ATTITUDE older than 200 ms - orientation invalid; a gap > 200 ms around t is invalid. */
void test_MAP14_attitude_timeout_and_gap(void)
{
    float p = 0.0f;
    float r = 0.0f;
    TEST_ASSERT_FALSE(mavrx_att_valid(&g_rx, 0u));
    feed_hb(false, 0u, 0u);
    feed_att(0.1f, 0.0f, 0u);
    feed_att(0.2f, 0.0f, 300u * MS); /* 300 ms gap */
    TEST_ASSERT_TRUE(mavrx_att_valid(&g_rx, 500u * MS));
    TEST_ASSERT_FALSE(mavrx_att_valid(&g_rx, 500u * MS + 1u));
    TEST_ASSERT_FALSE(mavrx_att_at(&g_rx, 150u * MS, 400u * MS, &p, &r));
    TEST_ASSERT_FALSE(mavrx_att_at(&g_rx, 300u * MS, 600u * MS, &p, &r));
    TEST_ASSERT_TRUE(mavrx_att_at(&g_rx, 300u * MS, 300u * MS, &p, &r));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.2f, p);
}

/* Roll interpolation across +-180 deg takes the short way. */
void test_MAP14_attitude_roll_wrap(void)
{
    float p = 0.0f;
    float r = 0.0f;
    feed_hb(false, 0u, 0u);
    feed_att(0.0f, 3.10f, 0u);
    feed_att(0.0f, -3.10f, 20u * MS);
    TEST_ASSERT_TRUE(mavrx_att_at(&g_rx, 10u * MS, 20u * MS, &p, &r));
    TEST_ASSERT_TRUE(fabsf(r) > 3.13f);
}

/* MAP-15: DISTANCE_SENSOR orientation 25 only, inside its range, signal not invalid, 500 ms. */
void test_MAP15_rangefinder(void)
{
    float h = 0.0f;
    feed_hb(false, 0u, 0u);
    feed_rf(FC_SYS, 150u, 0u, 100u, 0u); /* forward-facing: ignored */
    TEST_ASSERT_FALSE(mavrx_rf_valid(&g_rx, 0u, &h));
    feed_rf(FC_SYS, 150u, MAVRX_RF_ORIENTATION, 0u, 0u);
    /* MAP-15b: an out-of-range sample does not invalidate the height before 500 ms without a valid one. */
    feed_rf(FC_SYS, 800u, MAVRX_RF_ORIENTATION, 100u, 100u * MS);
    TEST_ASSERT_TRUE(mavrx_rf_valid(&g_rx, 500u * MS, &h));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.5f, h);
    TEST_ASSERT_FALSE(mavrx_rf_valid(&g_rx, 500u * MS + 1u, &h));
    feed_rf(FC_SYS, 800u, MAVRX_RF_ORIENTATION, 100u, 600u * MS); /* above max 700 */
    TEST_ASSERT_FALSE(mavrx_rf_valid(&g_rx, 600u * MS, &h));
    feed_rf(FC_SYS, 5u, MAVRX_RF_ORIENTATION, 100u, 700u * MS); /* below min 10 */
    TEST_ASSERT_FALSE(mavrx_rf_valid(&g_rx, 700u * MS, &h));
    feed_rf(FC_SYS, 150u, MAVRX_RF_ORIENTATION, 1u, 800u * MS); /* signal invalid */
    TEST_ASSERT_FALSE(mavrx_rf_valid(&g_rx, 800u * MS, &h));
}

/* FLT-03: COMMAND_ACK from the FC is recorded (for the log). */
void test_FLT03_command_ack_recorded(void)
{
    uint8_t b[MAVTX_BUF_LEN];
    mavlink_message_t m;
    feed_hb(true, 3u, 0u);
    (void)mavlink_msg_command_ack_pack_status(FC_SYS, 1u, &g_fc_status, &m, MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED,
                                              0u, 0, FC_SYS, 196u);
    mavrx_feed(&g_rx, b, mavlink_msg_to_send_buffer(b, &m), 5u * MS);
    TEST_ASSERT_TRUE(g_rx.ack_evt);
    TEST_ASSERT_EQUAL_UINT16(MAV_CMD_DO_SET_MODE, g_rx.ack_command);
    TEST_ASSERT_EQUAL_UINT8(MAV_RESULT_ACCEPTED, g_rx.ack_result);
    /* The ACK does not change the FC mode seen by the module. */
    TEST_ASSERT_EQUAL_UINT32(3u, g_rx.custom_mode);
}

/* ICD-04: COMMAND_LONG / COMMAND_INT to compid 196 are refused (queued for UNSUPPORTED), never
 * executed; broadcast or other targets are ignored; the queue is bounded. */
void test_ICD04_commands_refused(void)
{
    uint8_t b[MAVTX_BUF_LEN];
    feed_hb(true, TOF_COPTER_MODE_AUTO, 0u);
    const mavrx_t before = g_rx;
    mavrx_feed(&g_rx, b, cmd_long(b, 255u, 190u, FC_SYS, 196u, MAV_CMD_COMPONENT_ARM_DISARM), 1u * MS);
    mavrx_feed(&g_rx, b, cmd_long(b, 255u, 190u, FC_SYS, 0u, MAV_CMD_DO_SET_MODE), 1u * MS);   /* broadcast */
    mavrx_feed(&g_rx, b, cmd_long(b, 255u, 190u, FC_SYS, 1u, MAV_CMD_DO_SET_MODE), 1u * MS);   /* to the FC */
    mavrx_feed(&g_rx, b, cmd_long(b, 255u, 190u, 99u, 196u, MAV_CMD_DO_SET_MODE), 1u * MS);    /* other system */
    mavlink_message_t m;
    (void)mavlink_msg_command_int_pack_status(255u, 190u, &g_fc_status, &m, FC_SYS, 196u, 0u, MAV_CMD_DO_REPOSITION,
                                              0u, 0u, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f);
    mavrx_feed(&g_rx, b, mavlink_msg_to_send_buffer(b, &m), 1u * MS);
    TEST_ASSERT_EQUAL_UINT32(2u, g_rx.stats.cmds_refused);
    mavrx_nack_t n;
    TEST_ASSERT_TRUE(mavrx_take_nack(&g_rx, &n));
    TEST_ASSERT_EQUAL_UINT16(MAV_CMD_COMPONENT_ARM_DISARM, n.command);
    TEST_ASSERT_EQUAL_UINT8(255u, n.sysid);
    TEST_ASSERT_EQUAL_UINT8(190u, n.compid);
    TEST_ASSERT_TRUE(mavrx_take_nack(&g_rx, &n));
    TEST_ASSERT_EQUAL_UINT16(MAV_CMD_DO_REPOSITION, n.command);
    TEST_ASSERT_FALSE(mavrx_take_nack(&g_rx, &n));
    /* Nothing of the FC state changed. */
    TEST_ASSERT_EQUAL(before.armed, g_rx.armed);
    TEST_ASSERT_EQUAL_UINT32(before.custom_mode, g_rx.custom_mode);
    for (uint8_t k = 0; k < MAVRX_NACK_LEN + 2u; k++) {
        mavrx_feed(&g_rx, b, cmd_long(b, 255u, 190u, FC_SYS, 196u, 400u), 2u * MS);
    }
    TEST_ASSERT_EQUAL_UINT8(MAVRX_NACK_LEN, g_rx.nack_count);
    TEST_ASSERT_EQUAL_UINT32(2u, g_rx.stats.nack_dropped);
}

/* ---- ICD-03: fuzz ---- */

static uint32_t g_lcg = 12345u;
static uint8_t rnd8(void)
{
    g_lcg = g_lcg * 1664525u + 1013904223u;
    return (uint8_t)(g_lcg >> 24);
}

/* ICD-03: pure garbage never yields a message, the parser keeps going. */
void test_ICD03_fuzz_garbage(void)
{
    uint8_t b[256];
    for (uint32_t round = 0; round < 400u; round++) {
        for (uint16_t k = 0; k < sizeof b; k++) {
            b[k] = rnd8();
        }
        mavrx_feed(&g_rx, b, sizeof b, round * MS);
    }
    TEST_ASSERT_EQUAL_UINT32(400u * 256u, g_rx.stats.bytes);
    TEST_ASSERT_FALSE(g_rx.fc_known);
    TEST_ASSERT_TRUE(g_rx.stats.crc_errors + g_rx.stats.parse_errors > 0u);
    /* Still in sync afterwards: two heartbeats, the second one certainly parsed. */
    feed_hb(true, 4u, 500u * MS);
    feed_hb(true, 4u, 501u * MS);
    TEST_ASSERT_TRUE(g_rx.fc_known);
    TEST_ASSERT_EQUAL_UINT32(4u, g_rx.custom_mode);
}

/* ICD-03: valid packets between garbage without start bytes are all received, in any byte split. */
void test_ICD03_fuzz_garbage_between_packets(void)
{
    uint8_t pkt[MAVTX_BUF_LEN];
    uint8_t junk[64];
    uint32_t sent = 0u;
    for (uint32_t i = 0; i < 300u; i++) {
        const uint16_t n = fc_heartbeat(pkt, FC_SYS, 1u, MAV_TYPE_HEXAROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, false, i);
        const uint8_t nj = (uint8_t)(rnd8() % sizeof junk);
        for (uint8_t k = 0; k < nj; k++) {
            uint8_t v = rnd8();
            junk[k] = (v == MAVLINK_STX || v == MAVLINK_STX_MAVLINK1) ? 0u : v;
        }
        mavrx_feed(&g_rx, junk, nj, i * MS);
        /* Split the packet at a random point. */
        const uint16_t cut = (uint16_t)(rnd8() % n);
        mavrx_feed(&g_rx, pkt, cut, i * MS);
        mavrx_feed(&g_rx, pkt + cut, (uint32_t)(n - cut), i * MS);
        sent++;
    }
    TEST_ASSERT_EQUAL_UINT32(sent, g_rx.hb_count);
    TEST_ASSERT_EQUAL_UINT32(299u, g_rx.custom_mode);
}

/* ICD-03: garbage with start bytes, truncated and corrupted packets: bad frames dropped and counted,
 * no wrong data applied, most packets still received. */
void test_ICD03_fuzz_truncation_and_corruption(void)
{
    uint8_t pkt[MAVTX_BUF_LEN];
    uint8_t junk[40];
    uint32_t good = 0u;
    for (uint32_t i = 0; i < 500u; i++) {
        const uint16_t n = fc_heartbeat(pkt, FC_SYS, 1u, MAV_TYPE_HEXAROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, false, 77u);
        const uint8_t kind = rnd8() % 4u;
        if (kind == 0u) {
            mavrx_feed(&g_rx, pkt, (uint32_t)(rnd8() % n), i * MS); /* truncated */
        } else if (kind == 1u) {
            pkt[1u + rnd8() % (n - 1u)] ^= (uint8_t)(1u << (rnd8() % 8u)); /* one bit flipped */
            mavrx_feed(&g_rx, pkt, n, i * MS);
        } else {
            mavrx_feed(&g_rx, pkt, n, i * MS);
            good++;
        }
        const uint8_t nj = (uint8_t)(rnd8() % sizeof junk);
        for (uint8_t k = 0; k < nj; k++) {
            junk[k] = rnd8();
        }
        mavrx_feed(&g_rx, junk, nj, i * MS);
    }
    TEST_ASSERT_TRUE(g_rx.stats.crc_errors > 0u);
    TEST_ASSERT_TRUE(g_rx.hb_count >= good / 2u);
    TEST_ASSERT_TRUE(g_rx.hb_count <= good + 1u);
    TEST_ASSERT_EQUAL_UINT32(77u, g_rx.custom_mode);
}

/* ICD-03: a single flipped bit is a CRC error and changes nothing. */
void test_ICD03_bad_crc_counted(void)
{
    uint8_t pkt[MAVTX_BUF_LEN];
    feed_hb(false, 5u, 0u);
    const uint16_t n = fc_heartbeat(pkt, FC_SYS, 1u, MAV_TYPE_HEXAROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, true, 3u);
    pkt[n - 3u] ^= 0x01u;
    const uint32_t crc0 = g_rx.stats.crc_errors;
    mavrx_feed(&g_rx, pkt, n, 1u * MS);
    TEST_ASSERT_EQUAL_UINT32(crc0 + 1u, g_rx.stats.crc_errors);
    TEST_ASSERT_FALSE(g_rx.armed);
    TEST_ASSERT_EQUAL_UINT32(5u, g_rx.custom_mode);
}

/* Section 6: ATTITUDE faster than 100 Hz is decimated, the ring still covers >= 300 ms. */
void test_MAP14_attitude_ring_covers_300ms_at_any_rate(void)
{
    float p = 0.0f;
    float r = 0.0f;
    feed_hb(false, 0u, 0u);
    for (uint32_t k = 0; k <= 400u; k++) { /* 400 Hz for 1 s */
        feed_att(0.001f * (float)k, 0.0f, 1000u * MS + k * 2500u);
    }
    const uint32_t now = 2000u * MS;
    TEST_ASSERT_TRUE(mavrx_att_at(&g_rx, now - 300u * MS, now, &p, &r));
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.28f, p);
}

/* FLT-03: COMMAND_ACK addressed to another system or component is not taken. */
void test_FLT03_command_ack_for_others_ignored(void)
{
    uint8_t b[MAVTX_BUF_LEN];
    mavlink_message_t m;
    feed_hb(true, 3u, 0u);
    (void)mavlink_msg_command_ack_pack_status(FC_SYS, 1u, &g_fc_status, &m, MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED,
                                              0u, 0, 255u, 190u);
    mavrx_feed(&g_rx, b, mavlink_msg_to_send_buffer(b, &m), 5u * MS);
    TEST_ASSERT_FALSE(g_rx.ack_evt);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ICD01_obstacle_distance_fields);
    RUN_TEST(test_ICD01_heartbeat_fields_and_sequence);
    RUN_TEST(test_ICD01_statustext_cut_to_50);
    RUN_TEST(test_ICD01_set_message_interval);
    RUN_TEST(test_FLT05_hold_mode_only_brake_or_loiter);
    RUN_TEST(test_FLT03_copter_mode_numbers);
    RUN_TEST(test_ICD01_named_value_and_ack);
    RUN_TEST(test_ICD01_rx_fc_heartbeat);
    RUN_TEST(test_ICD01_rx_configured_sysid);
    RUN_TEST(test_MAP16_rx_link_timeouts_3s);
    RUN_TEST(test_ICD01_rx_foreign_data_ignored);
    RUN_TEST(test_MAP14_attitude_interpolation);
    RUN_TEST(test_MAP14_attitude_timeout_and_gap);
    RUN_TEST(test_MAP14_attitude_roll_wrap);
    RUN_TEST(test_MAP15_rangefinder);
    RUN_TEST(test_FLT03_command_ack_recorded);
    RUN_TEST(test_ICD04_commands_refused);
    RUN_TEST(test_ICD03_fuzz_garbage);
    RUN_TEST(test_ICD03_fuzz_garbage_between_packets);
    RUN_TEST(test_ICD03_fuzz_truncation_and_corruption);
    RUN_TEST(test_ICD03_bad_crc_counted);
    RUN_TEST(test_MAP14_attitude_ring_covers_300ms_at_any_rate);
    RUN_TEST(test_FLT03_command_ack_for_others_ignored);
    return UNITY_END();
}
