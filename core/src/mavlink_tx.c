#include "core/mavlink_tx.h"

#include <string.h>

/* REQ: ICD-01 */
void mavtx_init(mavtx_t *t, uint8_t sysid, uint8_t compid)
{
    memset(t, 0, sizeof *t);
    t->sysid = sysid;
    t->compid = compid;
}

static uint16_t finish(uint8_t *buf, const mavlink_message_t *msg)
{
    return mavlink_msg_to_send_buffer(buf, msg);
}

/* REQ: ICD-01, ICD-02 (field table of section 6) */
uint16_t mavtx_obstacle_distance(mavtx_t *t, uint8_t *buf, uint64_t time_usec,
                                 const uint16_t distances[TOF_NUM_SECTORS], uint16_t min_cm, uint16_t max_cm)
{
    mavlink_message_t msg;
    (void)mavlink_msg_obstacle_distance_pack_status(t->sysid, t->compid, &t->status, &msg, time_usec,
                                                    MAV_DISTANCE_SENSOR_LASER, distances, 0u, min_cm, max_cm,
                                                    MAVTX_OD_INCREMENT_F, MAVTX_OD_ANGLE_OFFSET,
                                                    MAV_FRAME_BODY_FRD);
    return finish(buf, &msg);
}

/* REQ: ICD-01, FLT-02 (CRITICAL) */
uint16_t mavtx_heartbeat(mavtx_t *t, uint8_t *buf, bool critical)
{
    mavlink_message_t msg;
    (void)mavlink_msg_heartbeat_pack_status(t->sysid, t->compid, &t->status, &msg, MAV_TYPE_ONBOARD_CONTROLLER,
                                            MAV_AUTOPILOT_INVALID, 0u, 0u,
                                            critical ? MAV_STATE_CRITICAL : MAV_STATE_ACTIVE);
    return finish(buf, &msg);
}

/* REQ: ICD-01, DIA-03 (<= 50 characters) */
uint16_t mavtx_statustext(mavtx_t *t, uint8_t *buf, uint8_t severity, const char *text)
{
    char field[MAVTX_TEXT_LEN];
    memset(field, 0, sizeof field);
    for (uint8_t k = 0; k < MAVTX_TEXT_LEN && text[k] != '\0'; k++) {
        field[k] = text[k];
    }
    mavlink_message_t msg;
    (void)mavlink_msg_statustext_pack_status(t->sysid, t->compid, &t->status, &msg, severity, field, 0u, 0u);
    return finish(buf, &msg);
}

/* REQ: ICD-01, FLT-05 (only SET_MESSAGE_INTERVAL and DO_SET_MODE are built) */
uint16_t mavtx_set_message_interval(mavtx_t *t, uint8_t *buf, uint8_t fc_sysid, uint32_t msg_id,
                                    int32_t interval_us)
{
    mavlink_message_t msg;
    (void)mavlink_msg_command_long_pack_status(t->sysid, t->compid, &t->status, &msg, fc_sysid,
                                               MAV_COMP_ID_AUTOPILOT1, MAV_CMD_SET_MESSAGE_INTERVAL, 0u,
                                               (float)msg_id, (float)interval_us, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    return finish(buf, &msg);
}

/* REQ: FLT-03, FLT-05 */
uint16_t mavtx_hold_mode(mavtx_t *t, uint8_t *buf, uint8_t fc_sysid, uint32_t custom_mode, uint8_t confirmation)
{
    if (custom_mode != TOF_COPTER_MODE_BRAKE && custom_mode != TOF_COPTER_MODE_LOITER) {
        return 0u; /* the module never commands any other mode */
    }
    mavlink_message_t msg;
    (void)mavlink_msg_command_long_pack_status(t->sysid, t->compid, &t->status, &msg, fc_sysid,
                                               MAV_COMP_ID_AUTOPILOT1, MAV_CMD_DO_SET_MODE, confirmation,
                                               (float)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)custom_mode, 0.0f,
                                               0.0f, 0.0f, 0.0f, 0.0f);
    return finish(buf, &msg);
}

/* REQ: ICD-01 (NAMED_VALUE_INT diagnostics) */
uint16_t mavtx_named_value_int(mavtx_t *t, uint8_t *buf, uint32_t time_boot_ms, const char *name, int32_t value)
{
    char field[10];
    memset(field, 0, sizeof field);
    for (uint8_t k = 0; k < sizeof field && name[k] != '\0'; k++) {
        field[k] = name[k];
    }
    mavlink_message_t msg;
    (void)mavlink_msg_named_value_int_pack_status(t->sysid, t->compid, &t->status, &msg, time_boot_ms, field, value);
    return finish(buf, &msg);
}

/* REQ: ICD-04 */
uint16_t mavtx_command_ack(mavtx_t *t, uint8_t *buf, uint16_t command, uint8_t result, uint8_t target_sys,
                           uint8_t target_comp)
{
    mavlink_message_t msg;
    (void)mavlink_msg_command_ack_pack_status(t->sysid, t->compid, &t->status, &msg, command, result, 0u, 0,
                                              target_sys, target_comp);
    return finish(buf, &msg);
}
