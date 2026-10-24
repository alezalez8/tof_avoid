#ifndef CORE_MAVLINK_RX_H
#define CORE_MAVLINK_RX_H

#include <stdbool.h>
#include <stdint.h>

#include "ardupilotmega/mavlink.h"

/* Incoming MAVLink from the FC (section 6, ICD-03, ICD-04). Only data is taken in: FC HEARTBEAT,
 * EXTENDED_SYS_STATE, ATTITUDE (ring buffer, interpolation at the capture time), DISTANCE_SENSOR
 * orientation 25 and COMMAND_ACK. Commands are never executed; COMMAND_LONG / COMMAND_INT addressed
 * to this component are answered UNSUPPORTED by the caller (mavrx_take_nack). Times are module times
 * of reception (now_us), never FC clocks. */

#define MAVRX_FC_COMPID 1u          /* MAV_COMP_ID_AUTOPILOT1 */
#define MAVRX_HB_TIMEOUT_US 3000000u  /* > 3 s: FC_LINK_LOST, PHASE_UNKNOWN */
#define MAVRX_ESS_TIMEOUT_US 3000000u /* > 3 s: PHASE_UNKNOWN */
#define MAVRX_ATT_TIMEOUT_US 200000u  /* > 200 ms: orientation invalid */
#define MAVRX_RF_TIMEOUT_US 500000u   /* > 500 ms: height invalid */
#define MAVRX_RF_ORIENTATION 25u      /* MAV_SENSOR_ROTATION_PITCH_270 (down) */

/* ATTITUDE ring: samples closer than 10 ms to the stored one are skipped, so 32 samples always cover
 * >= 310 ms whatever the stream rate (50 Hz requested: 640 ms). */
#define MAVRX_ATT_LEN 32u
#define MAVRX_ATT_MIN_STEP_US 10000u
#define MAVRX_NACK_LEN 4u

typedef struct {
    uint32_t t_us;
    float roll_rad;
    float pitch_rad;
} mavrx_att_t;

/* MAP-15c: state of the downward rangefinder. */
typedef enum {
    MAVRX_RF_FAULT = 0, /* above max_distance, invalid signal or no sample for > 500 ms (MAP-15b) */
    MAVRX_RF_OK,        /* min_distance < reading <= max_distance */
    MAVRX_RF_LOW        /* reading <= min_distance: the vehicle is at the ground */
} mavrx_rf_state_t;

typedef struct {
    uint16_t command;
    uint8_t sysid; /* requester */
    uint8_t compid;
} mavrx_nack_t;

typedef struct {
    uint32_t bytes;
    uint32_t msgs;          /* good frames */
    uint32_t crc_errors;    /* ICD-03: bad CRC (and bad signature) */
    uint32_t parse_errors;  /* framing errors reported by the parser */
    uint32_t cmds_refused;  /* ICD-04: COMMAND_LONG / COMMAND_INT to this component */
    uint32_t nack_dropped;  /* refusals not answered (queue full) */
    uint32_t foreign;       /* messages from other systems / components, ignored */
} mavrx_stats_t;

typedef struct {
    /* Parser state, own buffers (no global channel state). */
    mavlink_message_t rx_buf;
    mavlink_status_t rx_status;
    mavlink_message_t msg;
    mavlink_status_t msg_status;
    uint8_t last_parse_error;

    uint8_t own_compid;
    uint8_t cfg_sysid; /* MAV_SYSID: 0 - take the first FC HEARTBEAT */

    bool fc_known;
    uint8_t fc_sysid;

    /* HEARTBEAT FC */
    bool hb_seen;
    bool first_hb_evt; /* set once, on the first FC HEARTBEAT after start (FLT-08) */
    uint32_t t_hb_us;
    uint32_t hb_count; /* FC HEARTBEATs received: FLT-03 success needs one after the command */
    bool armed;
    uint32_t custom_mode;

    /* EXTENDED_SYS_STATE */
    bool ess_seen;
    uint32_t t_ess_us;
    uint8_t landed_state;

    /* ATTITUDE */
    mavrx_att_t att[MAVRX_ATT_LEN];
    uint8_t att_head;  /* next write position */
    uint8_t att_count;

    /* DISTANCE_SENSOR, orientation 25 */
    bool rf_seen;
    uint32_t t_rf_us;
    bool rf_in_range;    /* the last sample */
    bool rf_valid_seen;  /* a valid sample was received */
    uint32_t t_rf_valid_us;
    float rf_range_m;    /* the last valid sample */
    bool rf_low_seen;    /* MAP-15c: a sample at or below min_distance was received */
    uint32_t t_rf_low_us;
    bool rf_low_run;     /* the samples since t_rf_low_since_us are all below the minimum */
    uint32_t t_rf_low_since_us;

    /* COMMAND_ACK from the FC: logged, never a success criterion (FLT-03) */
    bool ack_evt;
    uint16_t ack_command;
    uint8_t ack_result;
    uint32_t t_ack_us;

    /* ICD-04: refusals to answer */
    mavrx_nack_t nack[MAVRX_NACK_LEN];
    uint8_t nack_count;

    mavrx_stats_t stats;
} mavrx_t;

void mavrx_init(mavrx_t *r, uint8_t own_compid, uint8_t cfg_sysid);

/* Feeds received UART bytes (any split, garbage, truncation). */
void mavrx_feed(mavrx_t *r, const uint8_t *data, uint32_t len, uint32_t now_us);

bool mavrx_hb_valid(const mavrx_t *r, uint32_t now_us);
bool mavrx_ess_valid(const mavrx_t *r, uint32_t now_us);

/* Newest ATTITUDE is not older than 200 ms. */
bool mavrx_att_valid(const mavrx_t *r, uint32_t now_us);

/* ATTITUDE interpolated at module time t_us (e.g. frame capture). False if the orientation is
 * invalid at now_us, t_us is older than the buffer, or the samples around t_us are > 200 ms apart.
 * A t_us newer than the newest sample holds it for up to 200 ms. */
bool mavrx_att_at(const mavrx_t *r, uint32_t t_us, uint32_t now_us, float *pitch_rad, float *roll_rad);

/* Rangefinder distance (m) of the last valid sample (inside its own min/max, signal not invalid) if
 * that sample is not older than 500 ms (MAP-15b: invalid for longer than 500 ms). */
bool mavrx_rf_valid(const mavrx_t *r, uint32_t now_us, float *range_m);

/* MAP-15c: the newer of the last valid and the last below-minimum sample, if not older than 500 ms;
 * otherwise MAVRX_RF_FAULT. A single sample above the maximum does not end a state before 500 ms.
 * range_m is set for MAVRX_RF_OK only. */
mavrx_rf_state_t mavrx_rf_state(const mavrx_t *r, uint32_t now_us, float *range_m);

/* MAP-15c: start of the current run of below-minimum samples (any other sample ends it); false if none. */
bool mavrx_rf_low_since(const mavrx_t *r, uint32_t now_us, uint32_t *since_us);

/* Takes one pending ICD-04 refusal; false if none. */
bool mavrx_take_nack(mavrx_t *r, mavrx_nack_t *out);

#endif /* CORE_MAVLINK_RX_H */
