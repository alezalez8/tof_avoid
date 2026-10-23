"""Scene generator checks without SITL (geometry of TZ 5.1 / 5.4, targets, protocol)."""

import math
import os
import random
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import scene  # noqa: E402
import sil_proto  # noqa: E402

LEVEL = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # n, e, d, roll, pitch, yaw on the ground (ground_d = 0)


def frame(cfg, objects, pose=LEVEL, sensor=1, st=None):
    rays = scene.precompute_rays(cfg)
    return scene.make_frame(cfg, rays[sensor], objects, pose, 0.0, st or scene.SensorState(), random.Random(1))


def quiet_cfg(**kw):
    j = {"noise_frac": 0.0, "noise_min_m": 0.0}
    j.update(kw)
    return scene.SensorConfig(j)


def test_zone_angles_match_tz_5_1():
    assert scene.zone_angles(4, 0, 0) == (-16.875, 16.875)
    assert scene.zone_angles(4, 3, 3) == (16.875, -16.875)
    d = scene.dir_sensor(0.0, 10.0)
    assert d[2] < 0.0  # elevation up -> z (down) negative


def test_ground_ranges_tz_5_4():
    """h = 0.20 m, level: bottom row centre ~0.66 m, second row ~2.0 m, lower FoV edge ~0.48 m."""
    h = 0.20
    assert abs(h / math.tan(math.radians(16.875)) - 0.66) < 0.01
    assert abs(h / math.tan(math.radians(5.625)) - 2.03) < 0.02
    cfg = quiet_cfg(sub=9, min_coverage=0.0, cluster_gap_m=10.0)
    _, truth = frame(cfg, [])
    bottom = [truth[12 + j] for j in range(4)]
    for z in bottom:
        assert z and z[0][1] == scene.GROUND_ID
        assert 0.48 <= z[0][0] <= 0.85
    for j in range(4):  # top two rows look above the horizon: no ground
        assert truth[j] == [] and truth[4 + j] == []


def test_box_ahead_is_seen_by_centre_sensor_middle_zones():
    cfg = quiet_cfg()
    box = scene.Box("b", 2.0, 2.5, -0.3, 0.3, 2.0)
    zones, truth = frame(cfg, [box])
    hit = [z for z in range(16) if any(t[1] == "b" for t in truth[z] if len(t) == 2)]
    assert 5 in hit and 6 in hit
    d_mm = [t[0] for t in zones[5][1]]
    assert min(abs(d - 2000) for d in d_mm) < 60


def test_left_sensor_sees_box_at_minus_30_deg():
    cfg = quiet_cfg()
    a = math.radians(-30.0)
    n, e = 2.0 * math.cos(a), 2.0 * math.sin(a)
    box = scene.Box("b", n - 0.2, n + 0.2, e - 0.2, e + 0.2, 2.0)
    _, truth_l = frame(cfg, [box], sensor=0)
    _, truth_c = frame(cfg, [box], sensor=1)
    assert any(t[1] == "b" for z in truth_l for t in z if len(t) == 2)
    assert not any(t[1] == "b" for z in truth_c for t in z if len(t) == 2)


def test_yaw_rotates_the_world():
    cfg = quiet_cfg()
    box = scene.Box("b", -0.3, 0.3, 2.0, 2.5, 2.0)  # east of the vehicle
    _, t0 = frame(cfg, [box])
    _, t90 = frame(cfg, [box], pose=(0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2))
    seen = lambda tr: any(t[1] == "b" for z in tr for t in z if len(t) == 2)  # noqa: E731
    assert not seen(t0) and seen(t90)


def test_two_targets_in_a_zone_closest_first():
    cfg = quiet_cfg()
    near = scene.Pole("near", 1.0, 0.18, 0.06, 2.0)   # thin pole in front of a wall
    wall = scene.Box("wall", 3.0, 3.2, -3.0, 3.0, 2.0)
    zones, truth = frame(cfg, [near, wall])
    two = [z for z in range(16) if len([t for t in truth[z] if len(t) == 2]) == 2]
    assert two
    z = two[0]
    assert truth[z][0][1] == "near" and truth[z][1][1] == "wall"
    assert zones[z][1][0][0] < zones[z][1][1][0]


def test_glare_marks_zones_invalid():
    cfg = quiet_cfg()
    st = scene.SensorState()
    st.invalid_frac = 1.0
    zones, _ = frame(cfg, [scene.Box("b", 2.0, 2.5, -1.0, 1.0, 2.0)], st=st)
    statuses = [t[3] for _, tg in zones for t in tg]
    assert statuses and all(s == 4 for s in statuses)


def test_phantom_target_lasts_n_frames():
    cfg = quiet_cfg()
    st = scene.SensorState()
    st.phantoms.append([5, 1.0, 2])
    rays = scene.precompute_rays(cfg)
    rng = random.Random(1)
    seen = []
    for _ in range(3):
        _, tr = scene.make_frame(cfg, rays[1], [], LEVEL, 0.0, st, rng)
        seen.append(any(t[1] == "phantom" for t in tr[5] if len(t) == 2))
    assert seen == [True, True, False]


def test_frame_encoding_layout():
    pkt = sil_proto.encode_frame(2, 4, 15, 7, [(3, [(1500, 6, 100, 5)])] * 16)
    magic, typ, sensor, res, freq, no, nz = struct.unpack_from("<IBBBBIB", pkt)
    assert (magic, typ, sensor, res, freq, no, nz) == (sil_proto.SIL_MAGIC, 1, 2, 4, 15, 7, 16)
    assert len(pkt) == 13 + 16 * 23


def test_clearance():
    box = scene.Box("b", 2.0, 3.0, -1.0, 1.0, 2.0)
    assert box.clearance(1.0, 0.0, 1.0) == 1.0
    assert box.clearance(2.5, 0.0, 1.0) == 0.0
    assert box.clearance(1.0, 0.0, 2.5) == float("inf")
