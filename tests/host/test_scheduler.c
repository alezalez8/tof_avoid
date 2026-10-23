#include <math.h>
#include <string.h>

#include "core/scheduler.h"
#include "unity.h"

#define MS 1000u
#define FC_SYS 7u
#define PERIOD_US 66667u /* 15 Hz frames */
#define RAW(i, j) ((uint8_t)((i) * 4u + (j)))

/* ---- simulated world: FC telemetry and three sensors ---- */

typedef struct {
    bool fc_on;
    bool armed;
    uint32_t mode;
    uint8_t ls;
    float pitch;
    uint16_t rf_cm; /* 0: no rangefinder */
    bool frames_on;
    uint8_t sensors_mask;
    uint8_t zone_c;  /* raw zone of sensor C with a target */
    int16_t r_c_mm;  /* 0: no target */
    uint8_t zone_c2;
    int16_t r_c2_mm;
} world_t;

typedef struct {
    uint32_t od;
    uint32_t hb;
    uint8_t hb_status;
    uint32_t set_mode;
    float set_mode_param2;
    uint32_t set_interval;
    float set_interval_ids[16];
    uint32_t ack;
    uint8_t ack_result;
    uint16_t ack_command;
    uint8_t ack_target;
    uint32_t nv;
    uint32_t text_n;
    char texts[32][51];
    uint16_t od_dist[TOF_NUM_SECTORS];
    mavlink_obstacle_distance_t od_msg;
    uint32_t other_cmds;
} wire_t;

static params_t g_p;
static sched_t g_s;
static world_t g_w;
static wire_t g_wire;
static uint32_t g_t;
static uint32_t g_t_frame;
static uint32_t g_frame_no;
static mavlink_status_t g_fc_status;
static mavlink_message_t g_pmsg;
static mavlink_status_t g_pst;
static mavlink_message_t g_pout;
static mavlink_status_t g_pst2;
static tof_frame_t g_fr;
static bool g_hb_once;
static bool g_ess_once;

static void parse_wire(const uint8_t *b, uint32_t n)
{
    for (uint32_t k = 0; k < n; k++) {
        if (mavlink_frame_char_buffer(&g_pmsg, &g_pst, b[k], &g_pout, &g_pst2) != MAVLINK_FRAMING_OK) {
            continue;
        }
        const mavlink_message_t *m = &g_pout;
        TEST_ASSERT_EQUAL_UINT32(FC_SYS, m->sysid);
        TEST_ASSERT_EQUAL_UINT8(196u, m->compid);
        switch (m->msgid) {
        case MAVLINK_MSG_ID_OBSTACLE_DISTANCE:
            g_wire.od++;
            mavlink_msg_obstacle_distance_decode(m, &g_wire.od_msg);
            memcpy(g_wire.od_dist, g_wire.od_msg.distances, sizeof g_wire.od_dist);
            break;
        case MAVLINK_MSG_ID_HEARTBEAT:
            g_wire.hb++;
            g_wire.hb_status = mavlink_msg_heartbeat_get_system_status(m);
            break;
        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t c;
            mavlink_msg_command_long_decode(m, &c);
            if (c.command == MAV_CMD_DO_SET_MODE) {
                g_wire.set_mode++;
                g_wire.set_mode_param2 = c.param2;
            } else if (c.command == MAV_CMD_SET_MESSAGE_INTERVAL) {
                g_wire.set_interval_ids[g_wire.set_interval % 16u] = c.param1;
                g_wire.set_interval++;
            } else {
                g_wire.other_cmds++;
            }
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_ACK: {
            mavlink_command_ack_t a;
            mavlink_msg_command_ack_decode(m, &a);
            g_wire.ack++;
            g_wire.ack_result = a.result;
            g_wire.ack_command = a.command;
            g_wire.ack_target = a.target_system;
            break;
        }
        case MAVLINK_MSG_ID_STATUSTEXT:
            if (g_wire.text_n < 32u) {
                memset(g_wire.texts[g_wire.text_n], 0, 51u);
                (void)mavlink_msg_statustext_get_text(m, g_wire.texts[g_wire.text_n]);
                g_wire.text_n++;
            }
            break;
        case MAVLINK_MSG_ID_NAMED_VALUE_INT:
            g_wire.nv++;
            break;
        default:
            g_wire.other_cmds++;
            break;
        }
    }
}

static bool text_seen(const char *prefix)
{
    for (uint32_t k = 0; k < g_wire.text_n; k++) {
        if (strncmp(g_wire.texts[k], prefix, strlen(prefix)) == 0) {
            return true;
        }
    }
    return false;
}

static void fc_send(const mavlink_message_t *m)
{
    uint8_t b[MAVTX_BUF_LEN];
    sched_uart_rx(&g_s, b, mavlink_msg_to_send_buffer(b, m), g_t);
}

static void fc_hb(void)
{
    mavlink_message_t m;
    (void)mavlink_msg_heartbeat_pack_status(FC_SYS, 1u, &g_fc_status, &m, MAV_TYPE_HEXAROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
                                            (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                                      (g_w.armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0u)),
                                            g_w.mode, MAV_STATE_ACTIVE);
    fc_send(&m);
}

static void fc_telemetry(void)
{
    mavlink_message_t m;
    if (!g_hb_once || g_t % (1000u * MS) == 0u) {
        g_hb_once = true;
        fc_hb();
    }
    if (!g_ess_once || g_t % (200u * MS) == 0u) {
        g_ess_once = true;
        (void)mavlink_msg_extended_sys_state_pack_status(FC_SYS, 1u, &g_fc_status, &m, 0u, g_w.ls);
        fc_send(&m);
    }
    if (g_t % (20u * MS) == 0u) {
        (void)mavlink_msg_attitude_pack_status(FC_SYS, 1u, &g_fc_status, &m, g_t / 1000u, 0.0f, g_w.pitch, 0.0f, 0.0f,
                                               0.0f, 0.0f);
        fc_send(&m);
    }
    if (g_w.rf_cm > 0u && g_t % (50u * MS) == 0u) {
        const float q[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        (void)mavlink_msg_distance_sensor_pack_status(FC_SYS, 1u, &g_fc_status, &m, g_t / 1000u, 10u, 700u, g_w.rf_cm,
                                                      0u, 0u, 25u, 0u, 0.0f, 0.0f, q, 100u);
        fc_send(&m);
    }
}

static void push_frames(void)
{
    for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
        if (((g_w.sensors_mask >> s) & 1u) == 0u) {
            continue;
        }
        memset(&g_fr, 0, sizeof g_fr);
        g_fr.sensor_id = s;
        g_fr.frame_no = g_frame_no;
        g_fr.t_ready_us = g_t;
        g_fr.res = 4u;
        g_fr.freq_hz = 15u;
        if (s == TOF_SENSOR_C) {
            if (g_w.r_c_mm > 0) {
                g_fr.zone[g_w.zone_c].nb_target_detected = 1u;
                g_fr.zone[g_w.zone_c].target[0].distance_mm = g_w.r_c_mm;
                g_fr.zone[g_w.zone_c].target[0].target_status = 5u;
            }
            if (g_w.r_c2_mm > 0) {
                g_fr.zone[g_w.zone_c2].nb_target_detected = 1u;
                g_fr.zone[g_w.zone_c2].target[0].distance_mm = g_w.r_c2_mm;
                g_fr.zone[g_w.zone_c2].target[0].target_status = 5u;
            }
        }
        TEST_ASSERT_EQUAL_INT(TOF_PUSH_OK, sched_push_frame(&g_s, &g_fr, g_t));
    }
    g_frame_no++;
}

static void drain(void)
{
    uint8_t b[512];
    uint32_t n;
    while ((n = sched_tx_take(&g_s, b, sizeof b, g_t)) > 0u) {
        parse_wire(b, n);
    }
    sched_tx_done(&g_s, g_t + 100u);
}

/* Runs the world for `ms` milliseconds in 10 ms loop passes (1 ms when frames are due). */
static void run(uint32_t ms)
{
    const uint32_t end = g_t + ms * MS;
    while (g_t < end) {
        g_t += MS;
        if (g_w.fc_on) {
            fc_telemetry();
        }
        if (g_w.frames_on && (uint32_t)(g_t - g_t_frame) >= PERIOD_US) {
            g_t_frame = g_t;
            push_frames();
        }
        if (g_t % (5u * MS) == 0u) {
            sched_step(&g_s, g_t);
            drain();
        }
    }
}

static void start(const params_t *p, bool defaults)
{
    memset(&g_w, 0, sizeof g_w);
    memset(&g_wire, 0, sizeof g_wire);
    memset(&g_fc_status, 0, sizeof g_fc_status);
    memset(&g_pst, 0, sizeof g_pst);
    g_t = 0u;
    g_t_frame = 0u;
    g_frame_no = 0u;
    g_hb_once = false;
    g_ess_once = false;
    sched_init(&g_s, p, defaults, 0u);
    g_w.fc_on = true;
    g_w.ls = MAV_LANDED_STATE_ON_GROUND;
    g_w.mode = 5u; /* LOITER */
    g_w.frames_on = true;
    g_w.sensors_mask = 0x7u;
}

void setUp(void)
{
    params_set_defaults(&g_p);
    (void)params_set_u32(&g_p, PARAM_ORIENT_CONFIRMED, 1u);
    start(&g_p, false);
}

void tearDown(void) {}

/* Brings the vehicle to FLIGHT in AUTO at 1.6 m body height. */
static void to_flight(float pitch_rad)
{
    run(10u); /* the first FC HEARTBEAT is on the ground, disarmed (otherwise FLT-08 applies) */
    g_w.armed = true;
    g_w.mode = TOF_COPTER_MODE_AUTO;
    g_w.ls = MAV_LANDED_STATE_IN_AIR;
    g_w.pitch = pitch_rad;
    g_w.rf_cm = (uint16_t)(1.6f / cosf(pitch_rad) * 100.0f + 1.0f);
    fc_hb();
    run(1500u);
    TEST_ASSERT_EQUAL_INT(PHASE_FLIGHT, g_s.phase.phase);
}

static uint16_t min_od(void)
{
    uint16_t m = 65535u;
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        m = (g_wire.od_dist[k] < m) ? g_wire.od_dist[k] : m;
    }
    return m;
}

/* MAP-12, DIA-06: every map has t1 <= t2 <= t3 <= t4 <= t5 in its log record. */
void test_MAP12_t1_t5_stamps(void)
{
    run(1000u);
    sched_map_log_t lg;
    bool any = false;
    for (int k = 0; k < 3; k++) {
        run(70u);
        while (sched_take_map_log(&g_s, &lg)) {
            any = true;
            if (lg.sent) {
                TEST_ASSERT_TRUE(lg.t1_us <= lg.t2_us);
                TEST_ASSERT_TRUE(lg.t2_us <= lg.t3_us);
                TEST_ASSERT_TRUE(lg.t3_us <= lg.t4_us);
                TEST_ASSERT_TRUE(lg.t4_us < lg.t5_us);
                TEST_ASSERT_TRUE(lg.t4_us - lg.t1_us <= 20000u); /* MAP-12 norm holds in this simulation */
                TEST_ASSERT_TRUE(lg.att_valid);
            } else {
                TEST_ASSERT_EQUAL_UINT32(0u, lg.t4_us);
            }
            TEST_ASSERT_TRUE(lg.seq > 0u);
            TEST_ASSERT_EQUAL_UINT8(PHASE_GROUND, lg.phase);
        }
    }
    TEST_ASSERT_TRUE(any);
}

/* ICD-02: one OBSTACLE_DISTANCE per fused map (rate limit above the map rate), never per sensor. */
void test_ICD02_one_message_per_map(void)
{
    (void)params_set_u32(&g_p, PARAM_MAV_DISTANCE_RATE_HZ, 30u);
    start(&g_p, false);
    run(1000u);
    const uint32_t maps0 = g_s.core.fusion.map_seq;
    const uint32_t od0 = g_wire.od;
    run(3000u);
    const uint32_t maps = g_s.core.fusion.map_seq - maps0;
    TEST_ASSERT_UINT32_WITHIN(1u, 45u, maps); /* 15 Hz, three sensors fused into one map */
    TEST_ASSERT_UINT32_WITHIN(1u, maps, g_wire.od - od0);
}

/* Section 6: OBSTACLE_DISTANCE at MAV_DISTANCE_RATE_HZ = 10 on average with 15 Hz maps. */
void test_ICD01_distance_rate_limit(void)
{
    run(1000u);
    const uint32_t od0 = g_wire.od;
    run(3000u);
    TEST_ASSERT_UINT32_WITHIN(2u, 30u, g_wire.od - od0);
    TEST_ASSERT_TRUE(g_s.stats.od_skipped_rate > 0u);
}

/* ICD-01: the packet on the wire carries the map by 5.2 with the section 6 constants. */
void test_ICD01_wire_packet(void)
{
    g_w.zone_c = RAW(0u, 1u);
    g_w.r_c_mm = 1500;
    run(1000u);
    TEST_ASSERT_TRUE(g_wire.od > 0u);
    TEST_ASSERT_EQUAL_UINT8(MAV_FRAME_BODY_FRD, g_wire.od_msg.frame);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, g_wire.od_msg.increment_f);
    TEST_ASSERT_EQUAL_UINT16(4u, g_wire.od_msg.min_distance);
    TEST_ASSERT_EQUAL_UINT16(300u, g_wire.od_msg.max_distance);
    TEST_ASSERT_UINT16_WITHIN(5u, 143u, min_od()); /* 1.5 m along a ray 16.9 deg up */
    TEST_ASSERT_EQUAL_UINT16(65535u, g_wire.od_dist[36]); /* behind: out of FoV */
    TEST_ASSERT_TRUE(g_wire.od_msg.time_usec > 900000u);
}

/* Section 6: no MAVLink output before the FC identity is known (MAV_SYSID = 0). */
void test_ICD01_silent_until_fc_known(void)
{
    g_w.fc_on = false;
    run(2000u);
    TEST_ASSERT_EQUAL_UINT32(0u, g_wire.od + g_wire.hb + g_wire.text_n);
    g_w.fc_on = true;
    run(1100u);
    TEST_ASSERT_TRUE(g_wire.hb >= 1u);
    TEST_ASSERT_TRUE(g_wire.od > 0u);
}

/* Section 6: HEARTBEAT 1 Hz ACTIVE; SET_MESSAGE_INTERVAL for the three streams at start and every 5 s
 * while a stream is missing. */
void test_ICD01_heartbeat_and_stream_requests(void)
{
    g_w.fc_on = false;
    g_w.frames_on = false;
    run(10u);
    fc_hb(); /* only the HEARTBEAT: no stream arrives yet */
    sched_step(&g_s, g_t);
    drain();
    TEST_ASSERT_EQUAL_UINT32(3u, g_wire.set_interval);
    TEST_ASSERT_EQUAL_FLOAT((float)MAVLINK_MSG_ID_ATTITUDE, g_wire.set_interval_ids[0]);
    TEST_ASSERT_EQUAL_FLOAT((float)MAVLINK_MSG_ID_DISTANCE_SENSOR, g_wire.set_interval_ids[1]);
    TEST_ASSERT_EQUAL_FLOAT((float)MAVLINK_MSG_ID_EXTENDED_SYS_STATE, g_wire.set_interval_ids[2]);
    g_w.fc_on = true;
    g_w.frames_on = true;
    run(10000u);
    /* ATTITUDE and EXTENDED_SYS_STATE arrive, DISTANCE_SENSOR not (rf_cm = 0): repeated every 5 s. */
    TEST_ASSERT_EQUAL_UINT32(5u, g_wire.set_interval);
    TEST_ASSERT_UINT32_WITHIN(1u, 10u, g_wire.hb);
    TEST_ASSERT_EQUAL_UINT8(MAV_STATE_ACTIVE, g_wire.hb_status);
    TEST_ASSERT_EQUAL_UINT32(0u, g_wire.other_cmds);
}

/* ICD-04: COMMAND_LONG to compid 196 is answered UNSUPPORTED on the wire, nothing else happens. */
void test_ICD04_unsupported_on_wire(void)
{
    run(1000u);
    mavlink_message_t m;
    (void)mavlink_msg_command_long_pack_status(255u, 190u, &g_fc_status, &m, FC_SYS, 196u,
                                               MAV_CMD_COMPONENT_ARM_DISARM, 0u, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                               0.0f);
    fc_send(&m);
    run(10u);
    TEST_ASSERT_EQUAL_UINT32(1u, g_wire.ack);
    TEST_ASSERT_EQUAL_UINT8(MAV_RESULT_UNSUPPORTED, g_wire.ack_result);
    TEST_ASSERT_EQUAL_UINT16(MAV_CMD_COMPONENT_ARM_DISARM, g_wire.ack_command);
    TEST_ASSERT_EQUAL_UINT8(255u, g_wire.ack_target);
    TEST_ASSERT_EQUAL_UINT32(0u, g_wire.set_mode);
}

/* MAP-17: on the ground (0.20 m) the ground seen by the bottom row is 65535, a real obstacle is
 * exported. */
void test_MAP17_ground_phase_export(void)
{
    g_w.zone_c = RAW(3u, 1u);
    g_w.r_c_mm = 689; /* centre ray of the bottom row meets the ground */
    run(1500u);
    TEST_ASSERT_EQUAL_INT(PHASE_GROUND, g_s.phase.phase);
    TEST_ASSERT_EQUAL_UINT16(65535u, min_od());
    TEST_ASSERT_EQUAL_UINT8(1u, g_s.core.fusion.frame[TOF_SENSOR_C].n_ground_zones);
    g_w.zone_c2 = RAW(0u, 1u);
    g_w.r_c2_mm = 2000;
    run(500u);
    TEST_ASSERT_UINT16_WITHIN(5u, 191u, min_od());
    /* No obstacle warning from the ground: only the real obstacle at 1.9 m. */
    TEST_ASSERT_TRUE(text_seen("TOF OBST WARN 1.9"));
    TEST_ASSERT_FALSE(text_seen("TOF OBST CRIT"));
}

/* MAP-17: in FLIGHT with KEEP the ground is exported as an obstacle (pitch -15 deg, 1.6 m). */
void test_MAP17_flight_keep_exports_ground(void)
{
    to_flight(-0.2618f);
    g_w.zone_c = RAW(3u, 1u);
    g_w.r_c_mm = 3030;
    run(500u);
    TEST_ASSERT_EQUAL_UINT8(1u, g_s.core.fusion.frame[TOF_SENSOR_C].n_ground_zones);
    TEST_ASSERT_TRUE(min_od() < 300u);
    sched_map_log_t lg;
    TEST_ASSERT_TRUE(sched_take_map_log(&g_s, &lg));
    TEST_ASSERT_EQUAL_UINT8(FUSION_VIEW_ALL, lg.view);
    /* LANDING: the same ground goes back to 65535. */
    g_w.ls = MAV_LANDED_STATE_LANDING;
    run(500u);
    TEST_ASSERT_EQUAL_INT(PHASE_LANDING, g_s.phase.phase);
    TEST_ASSERT_EQUAL_UINT16(65535u, min_od());
}

/* MAP-15b end to end: in the air without a valid rangefinder for > 500 ms the ground check is off
 * (every target real), IN_AIR gives FLIGHT; TOF_HEIGHT_GROUND_M is not used. */
void test_MAP15b_rangefinder_lost_in_air(void)
{
    run(10u);
    g_w.armed = true;
    g_w.mode = TOF_COPTER_MODE_AUTO;
    g_w.ls = MAV_LANDED_STATE_TAKEOFF;
    g_w.zone_c = RAW(3u, 1u);
    g_w.r_c_mm = 689; /* would be ground at 0.20 m */
    fc_hb();
    run(500u);
    TEST_ASSERT_EQUAL_INT(PHASE_TAKEOFF, g_s.phase.phase);
    TEST_ASSERT_EQUAL_UINT8(0u, g_s.core.fusion.frame[TOF_SENSOR_C].n_ground_zones);
    TEST_ASSERT_TRUE(min_od() < 300u); /* exported as a real obstacle */
    g_w.ls = MAV_LANDED_STATE_IN_AIR;
    run(300u);
    TEST_ASSERT_EQUAL_INT(PHASE_FLIGHT, g_s.phase.phase);
    /* Rangefinder present, then out of range above 7 m for > 500 ms: classification off again. */
    setUp();
    to_flight(-0.2618f);
    g_w.zone_c = RAW(3u, 1u);
    g_w.r_c_mm = 3030;
    run(300u);
    TEST_ASSERT_EQUAL_UINT8(1u, g_s.core.fusion.frame[TOF_SENSOR_C].n_ground_zones);
    g_w.rf_cm = 800u; /* above max_distance 700 */
    run(400u);
    TEST_ASSERT_EQUAL_UINT8(1u, g_s.core.fusion.frame[TOF_SENSOR_C].n_ground_zones);
    run(300u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_s.core.fusion.frame[TOF_SENSOR_C].n_ground_zones);
    TEST_ASSERT_EQUAL_INT(PHASE_FLIGHT, g_s.phase.phase);
}

/* FLT-05 end to end: armed in AUTO on the ground with a fault - no command; FLT-01/FLT-03: in FLIGHT
 * frames stop - stale after > 300 ms and DO_SET_MODE BRAKE on the same pass. */
void test_FLT03_end_to_end_stale_map(void)
{
    g_w.armed = true;
    g_w.mode = TOF_COPTER_MODE_AUTO;
    g_w.frames_on = false;
    run(2000u);
    TEST_ASSERT_TRUE((g_s.faults & PARAM_FAULT_BIT_MAP_STALE) != 0u);
    TEST_ASSERT_EQUAL_INT(PHASE_ARMED_GROUND, g_s.phase.phase);
    TEST_ASSERT_EQUAL_UINT32(0u, g_wire.set_mode);

    setUp();
    to_flight(0.0f);
    g_w.frames_on = false;
    const uint32_t t_stop = g_t_frame;
    while ((g_s.faults & PARAM_FAULT_BIT_MAP_STALE) == 0u) {
        run(5u);
    }
    TEST_ASSERT_TRUE(g_t - t_stop <= 300u * MS + 2u * PERIOD_US); /* last map <= 1 frame + loop after t_stop */
    TEST_ASSERT_TRUE(g_t - g_s.t_last_map_us > 300u * MS);
    TEST_ASSERT_TRUE(g_t - g_s.t_last_map_us <= 305u * MS);
    TEST_ASSERT_EQUAL_UINT32(1u, g_wire.set_mode); /* same pass: <= 100 ms */
    TEST_ASSERT_EQUAL_FLOAT(17.0f, g_wire.set_mode_param2);
    run(100u);
    TEST_ASSERT_TRUE(text_seen("TOF HOLD SENT BRAKE"));
    /* The FC switches to BRAKE: success by HEARTBEAT, no more commands. */
    g_w.mode = TOF_COPTER_MODE_BRAKE;
    fc_hb();
    run(3000u);
    TEST_ASSERT_EQUAL_INT(FAULT_SERIES_SUCCEEDED, g_s.fault.series);
    TEST_ASSERT_TRUE(g_wire.set_mode <= 2u);
}

/* FLT-06, FLT-03a end to end: the FC ignores the command - 6 transmissions, then "TOF HOLD REJECTED". */
void test_FLT06_end_to_end_rejected(void)
{
    to_flight(0.0f);
    g_w.frames_on = false;
    run(4000u);
    TEST_ASSERT_EQUAL_UINT32(6u, g_wire.set_mode);
    TEST_ASSERT_TRUE(text_seen("TOF HOLD REJECTED"));
    TEST_ASSERT_TRUE(text_seen("TOF DATA STALE"));
}

/* FLT-02: map LOST - no OBSTACLE_DISTANCE, HEARTBEAT CRITICAL, "TOF MAP LOST". */
void test_FLT02_lost_stops_distance(void)
{
    run(1000u);
    for (uint8_t k = 0; k < TOF_NUM_SENSORS; k++) {
        g_s.core.sh[k].sup = SH_FAILED;
        g_s.core.sh[k].t_reinit_us = g_t;
    }
    g_w.frames_on = false;
    const uint32_t od0 = g_wire.od;
    g_s.core.map_pending = true; /* a map that would be produced while LOST */
    run(1500u);
    TEST_ASSERT_EQUAL_INT(MAP_LOST, tof_core_map_state(&g_s.core));
    TEST_ASSERT_EQUAL_UINT32(od0, g_wire.od);
    TEST_ASSERT_TRUE(g_s.stats.od_skipped_lost >= 1u);
    TEST_ASSERT_EQUAL_UINT8(MAV_STATE_CRITICAL, g_wire.hb_status);
    TEST_ASSERT_TRUE(text_seen("TOF MAP LOST"));
    TEST_ASSERT_TRUE(text_seen("TOF SENSOR L LOST"));
}

/* FLT-08 end to end: first FC HEARTBEAT armed in AUTO after the module started. */
void test_FLT08_end_to_end_reboot(void)
{
    g_w.armed = true;
    g_w.mode = TOF_COPTER_MODE_RTL;
    g_w.ls = MAV_LANDED_STATE_IN_AIR;
    run(10u);
    TEST_ASSERT_EQUAL_UINT32(1u, g_wire.set_mode);
    run(200u);
    TEST_ASSERT_TRUE(text_seen("TOF REBOOTED IN FLIGHT"));
}

/* DIA-03, SNS-12a, SW-07: start-up messages - orientation not confirmed, parameters default, ready
 * with version. */
void test_DIA03_startup_messages(void)
{
    params_set_defaults(&g_p); /* ORIENT_CONFIRMED = 0 */
    start(&g_p, true);
    run(1500u);
    TEST_ASSERT_TRUE(text_seen("TOF ORIENT NOT CONFIRMED"));
    TEST_ASSERT_TRUE(text_seen("TOF PARAMS DEFAULT"));
    TEST_ASSERT_TRUE(text_seen("TOF READY "));
    TEST_ASSERT_FALSE(text_seen("TOF DATA STALE"));
    TEST_ASSERT_FALSE(text_seen("TOF PHASE UNKNOWN"));
    TEST_ASSERT_FALSE(text_seen("TOF FC LINK LOST"));
}

/* DIA-03: FC telemetry lost after it was seen - FC LINK LOST and PHASE UNKNOWN. */
void test_DIA03_fc_link_lost(void)
{
    run(1500u);
    g_w.fc_on = false;
    run(4000u);
    TEST_ASSERT_EQUAL_INT(PHASE_UNKNOWN, g_s.phase.phase);
    TEST_ASSERT_TRUE(text_seen("TOF FC LINK LOST"));
    TEST_ASSERT_TRUE(text_seen("TOF PHASE UNKNOWN"));
}

/* MAP-13: hold expiry is written to the log with the sector. */
void test_MAP13_hold_expiry_logged(void)
{
    g_w.zone_c = RAW(0u, 1u);
    g_w.r_c_mm = 1500;
    run(1000u);
    g_w.r_c_mm = 0; /* zone without a target: INV (CLEAR_MEASURED off) */
    run(1000u);
    ev_log_t r;
    bool found = false;
    while (sched_take_event_log(&g_s, &r)) {
        if (r.kind == EV_LOG_HOLD_EXPIRED) {
            found = true;
            TEST_ASSERT_TRUE(r.b < TOF_NUM_SECTORS);
        }
    }
    TEST_ASSERT_TRUE(found);
}

/* MAP-16: GROUND_RECOVER resets holds after the flight. */
void test_MAP16_ground_recover_resets_cells(void)
{
    to_flight(0.0f);
    g_w.zone_c = RAW(0u, 1u);
    g_w.r_c_mm = 1500;
    run(300u);
    g_w.frames_on = false;
    g_w.ls = MAV_LANDED_STATE_ON_GROUND;
    run(300u);
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        TEST_ASSERT_EQUAL_INT(CELL_UNKNOWN, g_s.core.fusion.cell[k].state);
        TEST_ASSERT_EQUAL_INT(CELL_UNKNOWN, g_s.core.fusion.cell_real[k].state);
    }
    TEST_ASSERT_EQUAL_INT(PHASE_ARMED_GROUND, g_s.phase.phase);
}

/* ICD-01: NAMED_VALUE_INT 1 Hz when enabled (three values), none by default. */
void test_ICD01_named_values(void)
{
    run(3000u);
    TEST_ASSERT_EQUAL_UINT32(0u, g_wire.nv);
    (void)params_set_u32(&g_p, PARAM_NAMED_VALUE_ENABLE, 1u);
    start(&g_p, false);
    run(3000u);
    TEST_ASSERT_UINT32_WITHIN(3u, 9u, g_wire.nv);
}

/* FLT-10: FAULT_WALL = 1 fills the sectors of a FAILED sensor with min_distance. */
void test_FLT10_wall_on_wire(void)
{
    (void)params_set_u32(&g_p, PARAM_FAULT_WALL, 1u);
    start(&g_p, false);
    run(1000u);
    g_s.core.sh[TOF_SENSOR_R].sup = SH_FAILED;
    g_s.core.sh[TOF_SENSOR_R].t_reinit_us = g_t;
    g_w.sensors_mask = 0x3u;
    run(500u);
    TEST_ASSERT_EQUAL_UINT16(4u, g_wire.od_dist[6]);
    TEST_ASSERT_EQUAL_UINT16(65535u, g_wire.od_dist[66]);
}

/* FLT-01: a TX queue overflow is an internal fault for 1 s; the app can report its own. */
void test_FLT01_internal_fault(void)
{
    run(1000u);
    for (int k = 0; k < 40; k++) {
        for (int c = 0; c < 4; c++) {
            mavlink_message_t m;
            (void)mavlink_msg_command_long_pack_status(255u, 190u, &g_fc_status, &m, FC_SYS, 196u, 400u, 0u, 0.0f,
                                                       0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            fc_send(&m);
        }
        sched_step(&g_s, g_t); /* UNSUPPORTED replies pile up: the queue is never drained */
    }
    TEST_ASSERT_TRUE(g_s.stats.tx_overflows > 0u);
    TEST_ASSERT_TRUE((g_s.faults & PARAM_FAULT_BIT_INTERNAL) != 0u);
    drain();
    run(1100u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_s.faults & PARAM_FAULT_BIT_INTERNAL);
    sched_set_internal_fault(&g_s, true);
    run(10u);
    TEST_ASSERT_TRUE((g_s.faults & PARAM_FAULT_BIT_INTERNAL) != 0u);
}

/* time_usec keeps counting over the 32-bit wrap of the module clock. */
void test_ICD01_time_usec_wraps(void)
{
    sched_init(&g_s, &g_p, false, 0xFFFFF000u);
    sched_step(&g_s, 0xFFFFF000u);
    sched_step(&g_s, 0x00000010u);
    TEST_ASSERT_EQUAL_UINT32(1u, g_s.time_hi);
}

/* Pitch ramp of 0.1 rad/s: every ATTITUDE sample carries the pitch of its own receive time. */
#define RAMP_RAD_PER_US 1.0e-7f

/* Runs 1 s of the ramp; every map log uses ATTITUDE at t1 - offset_us (5.1, DEV-12 p. 6). */
static void check_att_at_capture(uint32_t offset_us)
{
    sched_map_log_t lg;
    int n = 0;
    for (uint32_t k = 0; k < 1000u / 5u; k++) {
        g_w.pitch = RAMP_RAD_PER_US * (float)((g_t / (20u * MS) + 1u) * 20u * MS);
        run(5u);
        while (sched_take_map_log(&g_s, &lg)) {
            if (!lg.att_valid || lg.t1_us < offset_us + 40u * MS) {
                continue;
            }
            n++;
            /* No extrapolation: past the newest sample (20 ms grid, before t3) its value is held. */
            const uint32_t tc = lg.t1_us - offset_us;
            const uint32_t newest = lg.t3_us / (20u * MS) * 20u * MS;
            TEST_ASSERT_FLOAT_WITHIN(2.0e-4f, RAMP_RAD_PER_US * (float)(tc < newest ? tc : newest), lg.pitch_rad);
        }
    }
    TEST_ASSERT_TRUE(n > 5);
}

/* 5.1, DEV-12 p. 6: ATTITUDE is taken at mid-exposure, EXPOSURE_OFFSET_MS before the frame ready time;
 * 0 = half a frame period. */
void test_MAP14_attitude_at_mid_exposure(void)
{
    TEST_ASSERT_EQUAL_UINT32(100000u - 33333u, sched_capture_time_us(&g_s, 100000u));
    check_att_at_capture(33333u); /* 15 Hz: half of 66.7 ms; the 3.3 mrad shift exceeds the tolerance */

    params_t p = g_p;
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_set_u32(&p, PARAM_EXPOSURE_OFFSET_MS, 10u));
    start(&p, false);
    TEST_ASSERT_EQUAL_UINT32(90000u, sched_capture_time_us(&g_s, 100000u));
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu - 9999u, sched_capture_time_us(&g_s, 0u)); /* wraps like the clock */
    check_att_at_capture(10000u);

    p = g_p;
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_set_u32(&p, PARAM_SENSOR_FREQ_HZ, 10u));
    start(&p, false);
    TEST_ASSERT_EQUAL_UINT32(50000u, sched_capture_time_us(&g_s, 100000u));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_MAP12_t1_t5_stamps);
    RUN_TEST(test_ICD02_one_message_per_map);
    RUN_TEST(test_ICD01_distance_rate_limit);
    RUN_TEST(test_ICD01_wire_packet);
    RUN_TEST(test_ICD01_silent_until_fc_known);
    RUN_TEST(test_ICD01_heartbeat_and_stream_requests);
    RUN_TEST(test_ICD04_unsupported_on_wire);
    RUN_TEST(test_MAP17_ground_phase_export);
    RUN_TEST(test_MAP17_flight_keep_exports_ground);
    RUN_TEST(test_MAP15b_rangefinder_lost_in_air);
    RUN_TEST(test_FLT03_end_to_end_stale_map);
    RUN_TEST(test_FLT06_end_to_end_rejected);
    RUN_TEST(test_FLT02_lost_stops_distance);
    RUN_TEST(test_FLT08_end_to_end_reboot);
    RUN_TEST(test_DIA03_startup_messages);
    RUN_TEST(test_DIA03_fc_link_lost);
    RUN_TEST(test_MAP13_hold_expiry_logged);
    RUN_TEST(test_MAP16_ground_recover_resets_cells);
    RUN_TEST(test_ICD01_named_values);
    RUN_TEST(test_FLT10_wall_on_wire);
    RUN_TEST(test_FLT01_internal_fault);
    RUN_TEST(test_ICD01_time_usec_wraps);
    RUN_TEST(test_MAP14_attitude_at_mid_exposure);
    return UNITY_END();
}
