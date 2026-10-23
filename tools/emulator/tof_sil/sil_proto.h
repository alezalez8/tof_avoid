#ifndef TOF_SIL_PROTO_H
#define TOF_SIL_PROTO_H

/* UDP protocol scene.py -> tof_sil (one datagram per packet, little-endian, no padding).
 * Mirrored in tools/emulator/sil_proto.py; change both together.
 *
 * Header (5 bytes): magic u32 "TSIL", type u8.
 *
 * SIL_PKT_FRAME: sensor u8, res u8, freq_hz u8, frame_no u32, nzones u8, then nzones zones in raw ULD
 *   order (row-major), each SIL_ZONE_SIZE bytes:
 *     nb_target u8, ambient_per_spad u32,
 *     SIL_TARGETS x { distance_mm i16, range_sigma_mm u16, signal_per_spad u32, target_status u8 }.
 *   The frame is stamped t_ready_us on reception (the INT of the real module).
 *
 * SIL_PKT_SENSOR: sensor u8, powered u8, spi_errors u8.
 *   powered = 0: the sensor is off; a reinit attempt fails. spi_errors: driver errors to report now.
 *
 * SIL_PKT_CONTROL: cmd u8, arg u32 (see sil_cmd_t). */

#define SIL_MAGIC 0x4C495354u /* "TSIL" */
#define SIL_HDR_SIZE 5u
#define SIL_TARGETS 2u
#define SIL_TARGET_SIZE 9u
#define SIL_ZONE_SIZE (5u + SIL_TARGETS * SIL_TARGET_SIZE)
#define SIL_FRAME_HDR_SIZE (SIL_HDR_SIZE + 8u)
#define SIL_MAX_DATAGRAM 2048u

typedef enum {
    SIL_PKT_FRAME = 1,
    SIL_PKT_SENSOR = 2,
    SIL_PKT_CONTROL = 3
} sil_pkt_t;

typedef enum {
    SIL_CMD_RX_MUTE = 1,  /* arg ms: bytes FC -> module are dropped (cut RX wire) */
    SIL_CMD_TX_MUTE = 2,  /* arg ms: bytes module -> FC are dropped (cut TX wire) */
    SIL_CMD_INTERNAL = 3, /* arg 0/1: internal fault reported by the app (FLT-01) */
    SIL_CMD_QUIT = 4
} sil_cmd_t;

#endif /* TOF_SIL_PROTO_H */
