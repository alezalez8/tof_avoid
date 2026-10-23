"""T-12 (SITL): OBSTACLE_DISTANCE in the FC; prearm check of PRX (H-10).

Criterion (TZ 13): sectors on the right angles; 65535 and max + 1 correct; PRX Good. ICD-01, ICD-02.
a) default module parameters (CLEAR_MEASURED off): prearm without the module, PRX Good with it, poles at
   +30 deg / 2.0 m and -30 deg / 1.5 m on the ground and in hover, 65535 outside the FoV, one message per map;
b) CLEAR_MEASURED on (AMBIENT_CLEAR_MAX > 0), hover at 3 m: max + 1 in empty sectors is ignored by the FC.
"""

import math
import time

from common import H, Result, oa_db_items, ghost_items

POLE_R = 0.15
POLES = [("p_right", 30.0, 2.0), ("p_left", -30.0, 1.5)]
SCENE = {"objects": [{"type": "pole", "id": oid, "c": [d * math.cos(math.radians(az)), d * math.sin(math.radians(az))],
                      "r": POLE_R, "h": 6.0} for oid, az, d in POLES]}
EXPECT = {oid: (az, d - POLE_R) for oid, az, d in POLES}  # azimuth, horizontal distance to the surface
FOV_EDGE_DEG = 52.5 + 2.0  # outer zone edge of L/R + CAL_TOL_DEG


def sector_of(az):
    return int(round((az % 360.0) / 5.0)) % 72


def ang_diff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


ZONE_W_DEG = 45.0 / 4
CAL_TOL_DEG = 2.0


def footprint_half_deg(d):
    """Largest angle from the true azimuth at which the obstacle may appear (MAP-05: zone extent +
    CAL_TOL_DEG, sector half width, angular radius of the pole)."""
    return ZONE_W_DEG + CAL_TOL_DEG + 2.5 + math.degrees(math.asin(POLE_R / (d + POLE_R)))


def median(v):
    v = sorted(v)
    return v[len(v) // 2]


def check_map(res, label, ods, max_cm, clear_on):
    """ods: OBSTACLE_DISTANCE messages from the module (as forwarded to the GCS)."""
    m = ods[-1]
    res.check(label + ": frame BODY_FRD, increment 0 / 5.0, angle_offset 0, LASER",
              m.frame == 12 and m.increment == 0 and abs(m.increment_f - 5.0) < 1e-6 and m.angle_offset == 0
              and m.sensor_type == 0,
              "frame %d inc %d inc_f %.1f off %.1f type %d" % (m.frame, m.increment, m.increment_f, m.angle_offset,
                                                                m.sensor_type), "12 / 0 / 5.0 / 0 / 0")
    res.check(label + ": min/max distance", m.min_distance == 4 and m.max_distance == max_cm,
              "%d / %d" % (m.min_distance, m.max_distance), "4 / %d" % max_cm)
    outside = [k for k in range(72) if ang_diff(k * 5.0, 0.0) > FOV_EDGE_DEG + 2.5]
    bad = sum(1 for o in ods for k in outside if o.distances[k] != 65535)
    res.check(label + ": sectors outside the FoV = 65535 (%d sectors x %d maps)" % (len(outside), len(ods)), bad == 0,
              bad, 0)
    med = [median(o.distances[k] for o in ods) for k in range(72)]
    feet = set()
    for oid, (az, d) in EXPECT.items():
        k0 = sector_of(az)
        res.metric("%s_%s_sector%d_cm" % (label, oid, k0), med[k0], key=(label == "hover"))
        res.check("%s: %s sector %d (%+.0f deg) = %.0f cm" % (label, oid, k0, az, d * 100),
                  abs(med[k0] - d * 100) <= 10, med[k0], "%.0f +- 10" % (d * 100))
        foot = [k for k in range(72) if abs(med[k] - d * 100) <= 15]
        feet.update(foot)
        half = footprint_half_deg(d)
        worst = max(ang_diff(k * 5.0, az) for k in foot)
        res.metric("%s_%s_footprint_deg" % (label, oid), [k * 5 if k <= 36 else k * 5 - 360 for k in foot])
        res.check("%s: %s footprint within its zones (MAP-05)" % (label, oid), k0 in foot and worst <= half,
                  "%d sectors, max %.1f deg off" % (len(foot), worst), "<= %.1f deg" % half)
    if clear_on:
        free = [k for k in range(72) if ang_diff(k * 5.0, 0.0) <= 50.0 and k not in feet]
        clr = [k for k in free if med[k] == max_cm + 1]
        res.metric(label + "_clear_sectors_301", len(clr), key=True)
        res.check(label + ": every empty FoV sector = max + 1 (CLEAR_MEASURED)", free and len(clr) == len(free),
                  "%d of %d" % (len(clr), len(free)), "all, %d = %d cm" % (len(free), max_cm + 1))
    return med


def ap_faces(distances, min_cm, max_cm):
    """Face distances (cm) the FC must derive from one packet (AP_Proximity_MAV, PRX1_ORIENT/YAW_CORR 0)."""
    faces = {}
    for j, dcm in enumerate(distances):
        if dcm == 0 or dcm == 65535 or dcm < min_cm or dcm > max_cm:
            continue
        f = int(((j * 5.0 + 22.5) % 360.0) // 45.0)
        faces[f] = min(faces.get(f, 65535), dcm)
    return faces


def check_fc_view(res, label, g, since, med, max_cm):
    """FC interpretation: proximity faces (DISTANCE_SENSOR orientation 0..7 sent by the FC) against the faces that
    follow from the module packet; a swap, mirror or shift of sectors shows up here."""
    got = {}
    for t, m in g.history("DISTANCE_SENSOR", 1, since):
        if m.orientation <= 7:
            got.setdefault(m.orientation, []).append(m.current_distance)
    got = {k: median(v) for k, v in got.items()}
    exp = ap_faces(med, 4, max_cm)
    res.metric(label + "_fc_faces_cm", got)
    res.metric(label + "_faces_from_packet_cm", exp)
    for f in range(8):
        if f in exp:
            res.check("%s: FC face %d (%d deg) = packet" % (label, f, f * 45), abs(got.get(f, 0) - exp[f]) <= 5,
                      got.get(f), "%d +- 5" % exp[f])
        else:
            res.check("%s: FC face %d (%d deg): no obstacle" % (label, f, f * 45), got.get(f, max_cm) >= max_cm,
                      got.get(f), ">= %d (none)" % max_cm)
    res.check(label + ": right pole on a right face, left pole on a left face (no mirror)",
              exp.get(1, 65535) < 65535 and exp.get(7, 65535) < 65535 and got.get(1, 0) < max_cm and
              got.get(7, 0) < max_cm, "face1 %s, face7 %s" % (got.get(1), got.get(7)), "both obstacles")


def check_prx_log(res, label, st):
    logs = st.sitl.bin_logs()
    prx = H.bin_messages(logs[-1], ["PRX"]) if logs else []
    good = [p for p in prx if p.He == 2]
    res.check(label + ": PRX health Good in the FC log", prx and len(good) / len(prx) > 0.95,
              "%d/%d" % (len(good), len(prx)), "> 95 %")
    if good:
        can = median(p.CAn for p in good)
        cd = median(p.CDis for p in good)
        az, d = EXPECT["p_left"]
        half = footprint_half_deg(d)
        res.metric(label + "_prx_closest", {"CAn": can, "CDis": cd})
        res.check(label + ": PRX closest object = p_left (angle within its footprint)",
                  ang_diff(can, az) <= half and abs(cd - d) <= 0.12,
                  "%.1f deg / %.2f m" % (can, cd), "%.0f +- %.1f deg / %.2f +- 0.12 m" % (az % 360, half, d))


def test_T12_a_default():
    res = Result("T-12", "a", "OBSTACLE_DISTANCE в FC, prearm PRX (H-10), параметры модуля по умолчанию",
                 "Сектора на правильных углах; 65535 корректны; PRX Good; один OBSTACLE_DISTANCE на кадр карты")
    st = H.Stand("T-12_a", scene=SCENE)
    try:
        res.step("SITL + сцена без модуля; ожидание EKF; попытка arm")
        st.start(module=False, wait_ready=False)
        g = st.gcs
        g.wait(lambda: g.pos() is not None, 90, "EKF position")
        time.sleep(3)
        t_try = time.time()
        g.set_mode("LOITER")
        for _ in range(3):
            g.command(400, 1)
            time.sleep(1.5)
        txt = [x for x in g.texts(t_try - 1) if "PRX" in x[3]]
        res.check("H-10: arming refused by PRX prearm without the module", bool(txt) and not g.armed(),
                  txt[0][3] if txt else None, "'PreArm: PRX1: No Data'")
        s = g.latest("SYS_STATUS")
        res.check("PRX unhealthy without the module", not (s.onboard_control_sensors_health & H.PRX_BIT),
                  bool(s.onboard_control_sensors_health & H.PRX_BIT), False)

        res.step("Запуск модуля; ожидание PRX Good")
        t_mod = time.time()
        st.module.start()
        t_good = g.wait(lambda: (g.latest("SYS_STATUS").onboard_control_sensors_health & H.PRX_BIT) and time.time(),
                        30, "PRX Good")
        res.metric("prx_good_after_module_start_s", round(t_good - t_mod, 2), key=True)
        res.check("PRX Good after the module start", True, "%.1f s" % (t_good - t_mod), "-")
        st.wait_ready(60)

        res.step("На земле (disarmed): 10 с карты")
        t0 = time.time()
        time.sleep(10)
        ods = [m for t, m in g.history("OBSTACLE_DISTANCE", H.MODULE_COMPID, t0)]
        med = check_map(res, "ground", ods, 300, False)
        check_fc_view(res, "ground", g, t0, med, 300)

        res.step("Взлёт в GUIDED на 3 м, висение 10 с")
        g.rc_override(c3=1500, c7=2000)
        g.set_mode("GUIDED")
        g.arm()
        g.takeoff(3.0)
        time.sleep(3)
        t1 = time.time()
        time.sleep(10)
        t2 = time.time()
        ods = [m for t, m in g.history("OBSTACLE_DISTANCE", H.MODULE_COMPID, t1)]
        med = check_map(res, "hover", ods, 300, False)
        check_fc_view(res, "hover", g, t1, med, 300)

        res.step("ICD-02: один пакет на кадр карты")
        clk = st.module_clock(1)
        maps = [r for r in st.module_records("map") if t1 <= clk(r["t3"]) <= t2]
        sent = [r for r in maps if r["sent"]]
        n_gcs = len([1 for t, m in g.history("OBSTACLE_DISTANCE", H.MODULE_COMPID, t1) if t <= t2])
        res.metric("maps_in_window", len(maps))
        res.metric("od_sent_by_module", len(sent))
        res.metric("od_seen_by_gcs", n_gcs)
        seqs = [r["seq"] for r in sent]
        res.check("ICD-02: at most one OBSTACLE_DISTANCE per map (unique seq)", len(seqs) == len(set(seqs)),
                  "%d sent / %d unique" % (len(seqs), len(set(seqs))), "equal")
        res.check("ICD-02: every packet sent arrives (GCS count = module count +- 2)", abs(n_gcs - len(sent)) <= 2,
                  "%d / %d" % (n_gcs, len(sent)), "+- 2")
        rate = len(sent) / (t2 - t1)
        res.metric("od_rate_hz", round(rate, 2), key=True)
        res.check("OBSTACLE_DISTANCE rate = MAV_DISTANCE_RATE_HZ", 9.0 <= rate <= 10.5, round(rate, 2), "10 Hz")
        tu = [m.time_usec for t, m in g.history("OBSTACLE_DISTANCE", H.MODULE_COMPID, t1)]
        res.check("time_usec increases", all(b > a for a, b in zip(tu, tu[1:])), "monotonic", "monotonic")

        g.set_mode("LAND")
        g.wait(lambda: not g.armed(), 60, "landed and disarmed")
    finally:
        st.stop()
    check_prx_log(res, "hover", st)
    res.stand(st)
    assert res.save() == "PASS", res.d["checks"]


def test_T12_b_clear_measured():
    res = Result("T-12", "b", "OBSTACLE_DISTANCE: max + 1 (CLEAR_MEASURED включён)",
                 "max + 1 в пустых секторах FoV; FC их не принимает за препятствие; препятствия на своих углах")
    st = H.Stand("T-12_b", scene=SCENE, module_params={"AMBIENT_CLEAR_MAX": 1000})
    try:
        res.step("CLEAR_MEASURED: AMBIENT_CLEAR_MAX = 1000; взлёт в GUIDED на 3 м")
        st.start()
        g = st.gcs
        g.rc_override(c3=1500, c7=2000)
        g.set_mode("GUIDED")
        g.arm()
        g.takeoff(3.0)
        time.sleep(3)
        t1 = time.time()
        time.sleep(10)
        ods = [m for t, m in g.history("OBSTACLE_DISTANCE", H.MODULE_COMPID, t1)]
        med = check_map(res, "hover", ods, 300, True)
        check_fc_view(res, "hover", g, t1, med, 300)
        items = oa_db_items(g, t1)
        ghosts = ghost_items(items, SCENE)
        res.metric("oa_db_items_unique", len({it[5] for it in items}))
        res.check("OA database: no points from max + 1 sectors (only the poles)", items and not ghosts,
                  "%d reports of %d items, %d not at a pole" % (len(items), len({it[5] for it in items}), len(ghosts)),
                  "0 not at a pole")
        g.set_mode("LAND")
        g.wait(lambda: not g.armed(), 60, "landed and disarmed")
    finally:
        st.stop()
    res.stand(st)
    assert res.save() == "PASS", res.d["checks"]
