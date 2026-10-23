"""UDP protocol scene.py -> tof_sil; mirror of tools/emulator/tof_sil/sil_proto.h (change both together)."""

import struct

SIL_MAGIC = 0x4C495354  # "TSIL"
SIL_TARGETS = 2

PKT_FRAME = 1
PKT_SENSOR = 2
PKT_CONTROL = 3

CMD_RX_MUTE = 1
CMD_TX_MUTE = 2
CMD_INTERNAL = 3
CMD_QUIT = 4

_HDR = struct.Struct("<IB")
_FRAME_HDR = struct.Struct("<BBBIB")
_ZONE_HDR = struct.Struct("<BI")
_TARGET = struct.Struct("<hHIB")
_EMPTY_TARGET = _TARGET.pack(0, 0, 0, 255)


def encode_frame(sensor, res, freq_hz, frame_no, zones):
    """zones: res*res items in raw ULD order, each (ambient, [(distance_mm, sigma_mm, signal, status), ...])."""
    out = [_HDR.pack(SIL_MAGIC, PKT_FRAME), _FRAME_HDR.pack(sensor, res, freq_hz, frame_no & 0xFFFFFFFF, len(zones))]
    for ambient, targets in zones:
        targets = targets[:SIL_TARGETS]
        out.append(_ZONE_HDR.pack(len(targets), int(ambient)))
        for k in range(SIL_TARGETS):
            if k < len(targets):
                d, sg, sig, st = targets[k]
                out.append(_TARGET.pack(max(-32768, min(32767, int(d))), min(65535, int(sg)), int(sig), int(st)))
            else:
                out.append(_EMPTY_TARGET)
    return b"".join(out)


def encode_sensor(sensor, powered, spi_errors=0):
    return _HDR.pack(SIL_MAGIC, PKT_SENSOR) + struct.pack("<BBB", sensor, 1 if powered else 0, spi_errors)


def encode_control(cmd, arg=0):
    return _HDR.pack(SIL_MAGIC, PKT_CONTROL) + struct.pack("<BI", cmd, arg)
