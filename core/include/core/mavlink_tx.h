#ifndef CORE_MAVLINK_TX_H
#define CORE_MAVLINK_TX_H

#include <stdbool.h>
#include <stdint.h>

#include "ardupilotmega/mavlink.h"
#include "core/config.h"

/* Outgoing MAVLink of the module (section 6, ICD-01). Each function encodes one packet into buf
 * (MAVTX_BUF_LEN bytes) and returns its length, 0 if nothing was encoded. The only commands that can
 * be built are SET_MESSAGE_INTERVAL and DO_SET_MODE to BRAKE / LOITER (FLT-05). */

#define MAVTX_BUF_LEN MAVLINK_MAX_PACKET_LEN

/* ArduCopter custom modes (ardupilotmega COPTER_MODE). */
#define TOF_COPTER_MODE_AUTO 3u
#define TOF_COPTER_MODE_GUIDED 4u
#define TOF_COPTER_MODE_LOITER 5u
#define TOF_COPTER_MODE_RTL 6u
#define TOF_COPTER_MODE_BRAKE 17u

/* OBSTACLE_DISTANCE constants of the field table (section 6). */
#define MAVTX_OD_INCREMENT_F 5.0f
#define MAVTX_OD_ANGLE_OFFSET 0.0f

/* STATUSTEXT text length (no terminator when full). */
#define MAVTX_TEXT_LEN 50u

/* SET_MESSAGE_INTERVAL requests (section 6): ATTITUDE 50 Hz, DISTANCE_SENSOR 20 Hz,
 * EXTENDED_SYS_STATE 5 Hz. */
#define MAVTX_ATT_INTERVAL_US 20000
#define MAVTX_RF_INTERVAL_US 50000
#define MAVTX_ESS_INTERVAL_US 200000

typedef struct {
    mavlink_status_t status; /* own sequence counter, no global channel state */
    uint8_t sysid;
    uint8_t compid;
} mavtx_t;

void mavtx_init(mavtx_t *t, uint8_t sysid, uint8_t compid);

/* OBSTACLE_DISTANCE: time_usec module time, LASER, BODY_FRD, increment 0 / increment_f 5,
 * angle_offset 0, distances[72] in cm, MIN_DIST_CM / MAX_DIST_CM. */
uint16_t mavtx_obstacle_distance(mavtx_t *t, uint8_t *buf, uint64_t time_usec,
                                 const uint16_t distances[TOF_NUM_SECTORS], uint16_t min_cm, uint16_t max_cm);

/* HEARTBEAT: ONBOARD_CONTROLLER, autopilot INVALID, ACTIVE or CRITICAL. */
uint16_t mavtx_heartbeat(mavtx_t *t, uint8_t *buf, bool critical);

/* STATUSTEXT: text is cut to 50 characters. */
uint16_t mavtx_statustext(mavtx_t *t, uint8_t *buf, uint8_t severity, const char *text);

/* COMMAND_LONG MAV_CMD_SET_MESSAGE_INTERVAL to the FC autopilot. */
uint16_t mavtx_set_message_interval(mavtx_t *t, uint8_t *buf, uint8_t fc_sysid, uint32_t msg_id,
                                    int32_t interval_us);

/* COMMAND_LONG MAV_CMD_DO_SET_MODE (FLT-03): param1 = 1 (custom mode), param2 = BRAKE (17) or
 * LOITER (5). Any other mode is refused (returns 0). confirmation counts retries. */
uint16_t mavtx_hold_mode(mavtx_t *t, uint8_t *buf, uint8_t fc_sysid, uint32_t custom_mode, uint8_t confirmation);

/* NAMED_VALUE_INT (name cut to 10 characters). */
uint16_t mavtx_named_value_int(mavtx_t *t, uint8_t *buf, uint32_t time_boot_ms, const char *name, int32_t value);

/* COMMAND_ACK to a requester (ICD-04: MAV_RESULT_UNSUPPORTED). */
uint16_t mavtx_command_ack(mavtx_t *t, uint8_t *buf, uint16_t command, uint8_t result, uint8_t target_sys,
                           uint8_t target_comp);

#endif /* CORE_MAVLINK_TX_H */
