"""T-26 (SITL), BLOCKING: arm -> takeoff -> AUTO -> descent -> landing -> disarm; hypothesis H-14.

Criterion (TZ 13): phase transitions by 5.4; no false avoidance; after takeoff no ground points in the OA
database; an obstacle in the transition zone right after takeoff is detected and avoided; warnings persist
during landing. FC-05 (TZ 5.8): FLIGHT is confirmed before the first horizontal leg starts. MAP-15, MAP-15b,
MAP-15c, MAP-16, MAP-16a, MAP-17, MAP-19.

Takeoff altitude before the first horizontal leg = 3 m, the lowest allowed by FC-05 (the worst allowed case).
a) clean cycle, no obstacles, rangefinder valid on the ground (0.10 m);
b) a pole in the transition zone 3 m ahead of the takeoff point, landing 1.8 m in front of a box;
c) as a, rangefinder below its minimum on the ground (reads 0 when landed: MAP-15c);
d) as a with a leg at 8 m, above the 7 m rangefinder range (MAP-15b in flight);
e) LAND with a location: horizontal leg to the landing point with an obstacle on it;
f) rangefinder stuck below its minimum (every reading below RNGFND1_MIN): MAP-15c "TOF RF STUCK LOW" after
   RF_LOW_MAX_TAKEOFF_S in TAKEOFF, then MAP-15b.
"""

import statistics
import time

import math

from common import (H, Result, collisions, explained, ghost_exports, ghost_oa_items, ground_targets, landed_states,
                    oa_active, oa_db_items, path_deviation, phase_events, pose_at, module_texts)

ALT = 3.0  # FC-05: takeoff altitude before the first horizontal leg >= 3 m
TAKEOFF, WAYPOINT, LAND, CONDITION_YAW = 22, 16, 21, 115
EXPECTED_PHASES = ["GROUND", "ARMED_GROUND", "TAKEOFF", "FLIGHT", "LANDING", "GROUND_RECOVER", "ARMED_GROUND",
                   "GROUND"]
LS_NAMES = {0: "UNDEFINED", 1: "ON_GROUND", 2: "IN_AIR", 3: "TAKEOFF", 4: "LANDING"}


def fc_clock(g):
    """FC boot time (s) -> wall clock, from ATTITUDE time_boot_ms seen by the GCS."""
    off = [t - m.time_boot_ms / 1000.0 for t, m in g.history("ATTITUDE", 1)]
    o = statistics.median(off) if off else 0.0
    return lambda s: s + o


def run_cycle(res, st, mission, timeout=240.0):
    g = st.gcs
    res.step("Миссия: " + ", ".join("%s(%.0f,%.0f,%.1f)" % ({22: "TAKEOFF", 16: "WP", 21: "LAND", 115: "YAW"}[c], n, e, a)
                                     for c, n, e, a, _ in mission))
    g.upload_mission(mission)
    g.rc_override(c3=1500)
    g.set_mode("GUIDED")
    time.sleep(2)  # GROUND before arming
    g.arm()
    t_arm = time.time()
    g.set_mode("AUTO")
    res.step("arm (GUIDED) -> AUTO; ожидание посадки и disarm")
    g.wait(lambda: not g.armed(), timeout, "mission end (disarm)")
    time.sleep(4)  # GROUND after disarm
    return t_arm


def check_phases(res, st, g, t_arm):
    ev = phase_events(st)
    seq = [p for _, p, _ in ev]
    res.metric("phase_sequence", seq)
    res.check("MAP-16: phase sequence of the cycle (5.4)", seq[-len(EXPECTED_PHASES):] == EXPECTED_PHASES
              if len(seq) >= len(EXPECTED_PHASES) else False, " -> ".join(seq), " -> ".join(EXPECTED_PHASES))
    ls = landed_states(g, t_arm - 5)
    res.metric("landed_state_sequence", [LS_NAMES.get(v, v) for _, v in ls])
    t_of = lambda name: next((t for t, p, _ in ev if p == name), None)  # noqa: E731
    t_ls = lambda v: next((t for t, x in ls if x == v), None)  # noqa: E731
    # TAKEOFF follows LS = TAKEOFF, LANDING follows LS = LANDING, GROUND_RECOVER follows LS = ON_GROUND (FC data)
    for phase, lsv in (("TAKEOFF", 3), ("LANDING", 4)):
        tp, tl = t_of(phase), t_ls(lsv)
        lag = (tp - tl) if (tp and tl) else None
        res.metric("lag_%s_s" % phase.lower(), lag and round(lag, 2))
        res.check("MAP-16: %s entered after LS = %s (<= 0.5 s)" % (phase, LS_NAMES[lsv]),
                  lag is not None and -0.3 <= lag <= 0.5, lag and round(lag, 2), "0..0.5 s")
    tf = t_of("FLIGHT")
    res.check("MAP-16: FLIGHT entered", tf is not None, tf and round(tf - t_arm, 1), "after takeoff")
    return ev


def leg_start(st, t_from):
    """Start of the first horizontal leg: first pose 0.3 m away horizontally from the takeoff point (scene truth)."""
    poses = [r for r in st.scene.records() if r.get("k") == "pose" and r["t"] >= t_from]
    if not poses:
        return None
    n0, e0 = poses[0]["n"], poses[0]["e"]
    return next((r["t"] for r in poses if math.hypot(r["n"] - n0, r["e"] - e0) > 0.3), None)


def check_flight_before_leg(res, st, ev, t_arm):
    """FC-05: FLIGHT is confirmed before the first horizontal leg starts."""
    tf = next((t for t, p, _ in ev if p == "FLIGHT"), None)
    tl = leg_start(st, t_arm)
    margin = (tl - tf) if (tf and tl) else None
    res.metric("fc05_flight_before_leg_s", margin and round(margin, 2), key=True)
    res.check("FC-05: FLIGHT confirmed before the first horizontal leg", margin is not None and margin >= 0.0,
              margin and round(margin, 2), ">= 0 s")


def check_ground_free(res, st, g, ev, t_arm, objects=False):
    """No ground in the export and in the OA database after takeoff; no false avoidance."""
    t_to = next((t for t, p, _ in ev if p == "TAKEOFF"), t_arm)
    gh = ghost_exports(st, phases=["TAKEOFF", "FLIGHT", "LANDING"])
    by_phase = {}
    for x in gh:
        w = by_phase.setdefault(x["phase"], {"sector_maps": 0, "from_s": None, "to_s": None})
        w["sector_maps"] += 1
        w["from_s"] = w["from_s"] if w["from_s"] is not None else round(x["t"] - t_to, 2)
        w["to_s"] = round(x["t"] - t_to, 2)
    res.metric("ground_export_by_phase (s from TAKEOFF)", by_phase, key=True)
    if gh:
        res.metric("ground_export_examples", [dict(x, t=round(x["t"] - t_to, 2)) for x in gh[:6]])
    res.check("MAP-17: no ground exported as obstacle in TAKEOFF / FLIGHT / LANDING", not gh,
              "%d sector-maps" % len(gh), 0)
    # The OA database is fed only by this OBSTACLE_DISTANCE (PRX1_TYPE = 2, no other proximity source), so no
    # ground in the export means no ground in the database. Without scene objects any database item is ground.
    items = oa_db_items(g, t_to)
    uniq = {(x[5], round(x[1], 1), round(x[2], 1)) for x in items if x[4] > 0}
    res.metric("oa_db_items_after_takeoff", len(uniq), key=not st.scene.config.get("objects"))
    if not st.scene.config.get("objects"):
        res.check("T-26: no points in the OA database after takeoff (no objects in the scene)", not uniq,
                  "%d items" % len(uniq), 0)
    else:
        ghosts = ghost_oa_items(st, items)
        res.metric("oa_db_items_displaced_from_objects (FC attitude at reception, ADSB lag)", len(ghosts))
        res.check("T-26: no ground points in the OA database after takeoff (export without ground)", not gh,
                  "%d ground sector-maps exported" % len(gh), 0)
    act = [a for _, a in oa_active(st)]
    n_act = sum(1 for a in act if a)
    res.metric("oabr_records", len(act))
    res.metric("oabr_active_records", n_act, key=True)
    if not objects:
        res.check("T-26: no false avoidance (BendyRuler never active)", n_act == 0, "%d of %d OABR" % (n_act, len(act)),
                  0)
    return n_act


def h14(res, st, g, ev):
    """H-14: do takeoff, landing and the transition to the first horizontal leg use OA, and do they need data the
    module excludes in TAKEOFF / LANDING (GROUND_SUSPECT -> 65535)?

    4.7.2: AUTO takeoff runs auto_takeoff (no OA); LAND with a location flies there as a WP leg (LS = IN_AIR) and
    only then descends (LS = LANDING, land controller, no OA steering; the planner thread keeps logging OABR with
    Act = 0). So: (A) BendyRuler is never active while LS = TAKEOFF / LANDING; (B) while the FC already flies
    horizontally (LS = IN_AIR) and the module still excludes ground (TAKEOFF / LANDING), the scene truth has no
    ground within MAX_DIST. EXTENDED_SYS_STATE comes at 5 Hz: records within 0.3 s of a change are not attributed."""
    fcc = fc_clock(g)
    oabr = [(fcc(s), a) for s, a in oa_active(st)]
    ls = landed_states(g)
    spans = [(v, t0, nxt[0]) for (t0, v), nxt in zip(ls, ls[1:] + [(time.time() + 1e3, None)])]
    in_ls = {}
    for t, a in oabr:
        sp = next((x for x in spans if x[1] <= t < x[2]), None)
        k = LS_NAMES.get(sp[0], "?") if sp and t - sp[1] > 0.3 and sp[2] - t > 0.3 else "edge"
        in_ls.setdefault(k, [0, 0])
        in_ls[k][0] += 1
        in_ls[k][1] += 1 if a else 0
    res.metric("h14_oabr_records_by_landed_state_[all,active]", in_ls)
    # The planner thread may keep evaluating (OABR Act = 1 with an obstacle near the landing point); what matters is
    # whether the vehicle follows it: horizontal travel while LS = TAKEOFF / LANDING (edges excluded).
    poses = [r for r in st.scene.records() if r.get("k") == "pose"]
    travel = {}
    for v, a0, a1 in spans:
        if v not in (3, 4):
            continue
        ps = [p for p in poses if a0 + 0.3 <= p["t"] <= a1 - 0.3]
        if ps:
            d = max(math.hypot(p["n"] - ps[0]["n"], p["e"] - ps[0]["e"]) for p in ps)
            travel[LS_NAMES[v]] = round(max(travel.get(LS_NAMES[v], 0.0), d), 2)
    res.metric("h14_horizontal_travel_m_in_LS_TAKEOFF_LANDING", travel)
    worst = max(travel.values(), default=0.0)
    res.check("H-14 (A): OA does not steer while LS = TAKEOFF / LANDING (horizontal travel < 0.3 m)", worst < 0.3,
              travel, "< 0.3 m")
    spans_m = [(p, t0, nxt[0]) for (t0, p, _), nxt in zip(ev, ev[1:] + [(time.time() + 1e3, None, None)])]
    win = []
    for p, m0, m1 in spans_m:
        if p not in ("TAKEOFF", "LANDING"):
            continue
        for v, l0, l1 in spans:
            if v == 2 and max(m0, l0) < min(m1, l1):
                win.append((p, max(m0, l0), min(m1, l1)))
    n_g = sum(ground_targets(st, t0, t1) for _, t0, t1 in win)
    res.metric("h14_windows_LS_IN_AIR_module_excludes_ground_s", [(p, round(t1 - t0, 2)) for p, t0, t1 in win],
               key=True)
    res.metric("h14_ground_zones_within_max_in_windows", n_g)
    res.check("H-14 (B): no excluded ground within MAX_DIST while the FC flies horizontally in module TAKEOFF / "
              "LANDING", n_g == 0, "%d zones in %.2f s" % (n_g, sum(t1 - t0 for _, t0, t1 in win)), 0)


def test_T26_a_clean_cycle():
    res = Result("T-26", "a", "Цикл arm → взлёт → AUTO → снижение → посадка → disarm, без препятствий",
                 "Переходы фаз по 5.4; нет ложного обхода; после взлёта в базе OA нет точек земли")
    st = H.Stand("T-26_a", scene={"objects": []})
    try:
        st.start()
        mission = [(TAKEOFF, 0, 0, ALT, 0), (WAYPOINT, 10, 0, ALT, 0), (WAYPOINT, 10, 6, ALT, 0),
                   (WAYPOINT, 0, 6, 1.6, 0), (LAND, 0, 6, 0, 0)]
        t_arm = run_cycle(res, st, mission)
        g = st.gcs
        ev = check_phases(res, st, g, t_arm)
        check_flight_before_leg(res, st, ev, t_arm)
        check_ground_free(res, st, g, ev, t_arm)
        h14(res, st, g, ev)
        dev = path_deviation(st.scene.records(), (0, 0), (10, 0), t_arm)
        res.metric("first_leg_max_lateral_dev_m", round(dev, 2))
        res.check("no false avoidance: first leg straight (lateral dev < 0.5 m)", dev < 0.5, round(dev, 2), "< 0.5 m")
    finally:
        st.stop()
    res.stand(st)
    assert res.save() == "PASS", [c for c in res.d["checks"] if not c["ok"]]


def test_T26_b_transition_obstacle_and_landing():
    res = Result("T-26", "b", "Препятствие в зоне перехода сразу после взлёта; посадка перед препятствием",
                 "Препятствие обнаруживается и обходится; при посадке предупреждения сохраняются; земли в базе OA нет")
    scene = {"objects": [{"type": "pole", "id": "pole", "c": [3.0, 0.0], "r": 0.2, "h": 6.0},
                         {"type": "box", "id": "box", "n": [9.8, 10.4], "e": [5.4, 6.6], "h": [0, 2.5]}]}
    st = H.Stand("T-26_b", scene=scene)
    try:
        st.start()
        # CONDITION_YAW 0 deg runs with LAND: the box (north of the landing point) stays in the FoV
        mission = [(TAKEOFF, 0, 0, ALT, 0), (WAYPOINT, 10, 0, ALT, 0), (WAYPOINT, 4, 6, ALT, 0),
                   (WAYPOINT, 8, 6, ALT, 0), (CONDITION_YAW, 0, 0, 0, 0), (LAND, 8, 6, 0, 0)]
        t_arm = run_cycle(res, st, mission)
        g = st.gcs
        ev = check_phases(res, st, g, t_arm)
        check_flight_before_leg(res, st, ev, t_arm)
        n_act = check_ground_free(res, st, g, ev, t_arm, objects=True)
        h14(res, st, g, ev)
        sc = st.scene.records()
        clr, hits, best = collisions(sc, t_from=t_arm)
        res.metric("min_clearance_m", round(clr, 2), key=True)
        res.metric("min_clearance_at", best)
        res.check("no collision (clearance > vehicle radius %.2f m)" % H.VEHICLE_RADIUS_M, not hits, round(clr, 2),
                  "> %.2f m" % H.VEHICLE_RADIUS_M)
        pole_clr = min((r["clr"] for r in sc if r.get("k") == "pose" and r["t"] > t_arm and r.get("obj") == "pole"
                        and r["clr"] is not None), default=None)
        dev = path_deviation(sc, (0, 0), (10, 0), t_arm)
        res.metric("first_leg_max_lateral_dev_m", round(dev, 2))
        res.metric("pole_min_clearance_m", pole_clr)
        res.check("transition obstacle avoided: BendyRuler active and the track bends", n_act > 0 and dev > 0.5,
                  "OABR active %d, dev %.2f m" % (n_act, dev), "active, dev > 0.5 m")
        # warnings in LANDING: the box stays exported and a warning is active
        import scene as S
        clk = st.module_clock(1)
        t_land = next((t for t, p, _ in ev if p == "LANDING"), None)
        t_gr = next((t for t, p, _ in ev if p == "GROUND_RECOVER"), None)
        box = [S.make_object(o) for o in scene["objects"] if o["id"] == "box"]
        poses = [r for r in sc if r.get("k") == "pose"]
        land_maps = [r for r in st.module_records("map") if r["ph"] == 5 and r["sent"]]
        with_box = []
        in_view = []
        for r in land_maps:
            t = clk(r["t3"])
            p = pose_at(poses, t)
            bb = math.degrees(math.atan2(6.0 - p["e"], 9.8 - p["n"]) - p["yaw"])
            if abs((bb + 180.0) % 360.0 - 180.0) > 45.0:
                continue  # box outside the FoV at this pose
            in_view.append(r)
            if any(4 <= dcm <= 300 and explained(box, poses, t, dcm / 100.0,
                                                 lambda q, k=k: q["yaw"] + math.radians(k * 5.0))
                   for k, dcm in enumerate(r["d"])):
                with_box.append(r)
        res.metric("landing_maps", len(land_maps))
        res.metric("landing_maps_box_in_fov", len(in_view))
        res.metric("landing_maps_with_box", len(with_box))
        res.check("MAP-17: real obstacle in the FoV exported during LANDING",
                  len(in_view) >= 10 and len(with_box) / len(in_view) > 0.9,
                  "%d/%d (box in FoV)" % (len(with_box), len(in_view)), "> 90 %, >= 10 maps")
        tx = [(clk(r["t"]), r["text"]) for r in module_texts(st)]
        last = None
        for t, txt in tx:
            if t <= (t_gr or time.time()) and txt.startswith("TOF OBST"):
                last = (t, txt)
        warn_ok = last is not None and not last[1].startswith("TOF OBST CLEAR")
        cleared_in_landing = [txt for t, txt in tx if t_land and t_gr and t_land <= t <= t_gr and "CLEAR" in txt]
        res.metric("obstacle_texts", [(round(t - t_arm, 1), x) for t, x in tx if x.startswith("TOF OBST")])
        res.check("DIA-03: obstacle warning active through LANDING (no CLEAR)", warn_ok and not cleared_in_landing,
                  last and last[1], "WARN/CRIT kept")
        gcs_txt = [x for x in g.texts(t_arm, H.MODULE_COMPID) if x[3].startswith("TOF OBST")]
        res.check("warnings reach the GCS", bool(gcs_txt), len(gcs_txt), ">= 1")
    finally:
        st.stop()
    res.stand(st)
    assert res.save() == "PASS", [c for c in res.d["checks"] if not c["ok"]]


def test_T26_c_rangefinder_invalid_on_ground():
    res = Result("T-26", "c", "Цикл без препятствий; дальномер на земле ниже минимума (MAP-15c)",
                 "Как T-26 a: переходы фаз; нет ложного обхода; после взлёта нет точек земли в базе OA")
    st = H.Stand("T-26_c", scene={"objects": []}, fc_params={"SIM_SONAR_POS_Z": 0.0},
                 module_params={"RF_Z_M": 0.20})
    try:
        st.start()
        mission = [(TAKEOFF, 0, 0, ALT, 0), (WAYPOINT, 10, 0, ALT, 0), (WAYPOINT, 10, 6, ALT, 0),
                   (WAYPOINT, 0, 6, 1.6, 0), (LAND, 0, 6, 0, 0)]
        t_arm = run_cycle(res, st, mission)
        g = st.gcs
        ev = check_phases(res, st, g, t_arm)
        check_flight_before_leg(res, st, ev, t_arm)
        check_ground_free(res, st, g, ev, t_arm)
        h14(res, st, g, ev)
        dev = path_deviation(st.scene.records(), (0, 0), (10, 0), t_arm)
        res.metric("first_leg_max_lateral_dev_m", round(dev, 2))
        res.check("no false avoidance: first leg straight (lateral dev < 0.5 m)", dev < 0.5, round(dev, 2), "< 0.5 m")
    finally:
        st.stop()
    res.stand(st)
    assert res.save() == "PASS", [c for c in res.d["checks"] if not c["ok"]]


def test_T26_d_above_rangefinder_range():
    res = Result("T-26", "d", "Цикл с участком на 8 м (выше диапазона дальномера 7 м, MAP-15b в полёте)",
                 "Фаза FLIGHT без валидной высоты; переходы фаз; нет ложного обхода и точек земли")
    st = H.Stand("T-26_d", scene={"objects": []})
    try:
        st.start()
        mission = [(TAKEOFF, 0, 0, ALT, 0), (WAYPOINT, 6, 0, ALT, 0), (WAYPOINT, 6, 0, 8.0, 0),
                   (WAYPOINT, 6, 8, 8.0, 0), (WAYPOINT, 0, 8, 2.0, 0), (LAND, 0, 8, 0, 0)]
        t_arm = run_cycle(res, st, mission, timeout=300)
        g = st.gcs
        ev = check_phases(res, st, g, t_arm)
        check_flight_before_leg(res, st, ev, t_arm)
        check_ground_free(res, st, g, ev, t_arm)
        sts = [r for r in st.module_records("st") if r["ph"] == 4]
        high = [r for r in sts if not r["rf"]]
        res.metric("flight_status_records_without_rangefinder", len(high))
        res.check("MAP-15b: FLIGHT kept with the rangefinder out of range", len(high) >= 5 and
                  all(r["ph"] == 4 for r in high), len(high), ">= 5 s in FLIGHT without valid height")
    finally:
        st.stop()
    res.stand(st)
    assert res.save() == "PASS", [c for c in res.d["checks"] if not c["ok"]]


def test_T26_e_land_with_location():
    res = Result("T-26", "e", "Посадка LAND с координатами: горизонтальный участок в LS = LANDING, препятствие на нём",
                 "H-14: участок посадки с OA; реальное препятствие обнаруживается и обходится; земли в экспорте нет")
    scene = {"objects": [{"type": "box", "id": "box", "n": [3.6, 4.2], "e": [5.4, 6.6], "h": [0, 2.5]}]}
    st = H.Stand("T-26_e", scene=scene)
    try:
        st.start()
        mission = [(TAKEOFF, 0, 0, ALT, 0), (WAYPOINT, 0, 6, ALT, 0), (LAND, 8, 6, 0, 0)]
        t_arm = run_cycle(res, st, mission)
        g = st.gcs
        ev = check_phases(res, st, g, t_arm)
        check_flight_before_leg(res, st, ev, t_arm)
        n_act = check_ground_free(res, st, g, ev, t_arm, objects=True)
        h14(res, st, g, ev)
        sc = st.scene.records()
        clr, hits, best = collisions(sc, t_from=t_arm)
        res.metric("min_clearance_m", round(clr, 2), key=True)
        res.check("no collision (clearance > vehicle radius %.2f m)" % H.VEHICLE_RADIUS_M, not hits, round(clr, 2),
                  "> %.2f m" % H.VEHICLE_RADIUS_M)
        dev = path_deviation(sc, (0, 6), (8, 6), t_arm)
        res.metric("land_leg_max_lateral_dev_m", round(dev, 2))
        res.check("obstacle on the LAND leg avoided: BendyRuler active and the track bends", n_act > 0 and dev > 0.5,
                  "OABR active %d, dev %.2f m" % (n_act, dev), "active, dev > 0.5 m")
    finally:
        st.stop()
    res.stand(st)
    assert res.save() == "PASS", [c for c in res.d["checks"] if not c["ok"]]


def test_T26_f_rangefinder_stuck_low():
    res = Result("T-26", "f", "Дальномер залип ниже минимума (все отсчёты < RNGFND1_MIN): MAP-15c, «TOF RF STUCK LOW»",
                 "MAP-15c: в TAKEOFF ниже минимума дольше RF_LOW_MAX_TAKEOFF_S (5 с) — MAP-15b (FLIGHT) и «TOF RF STUCK LOW»")
    st = H.Stand("T-26_f", scene={"objects": []}, fc_params={"RNGFND1_MIN": 8.5, "RNGFND1_MAX": 9.0})
    try:
        st.start()
        mission = [(TAKEOFF, 0, 0, ALT, 0), (WAYPOINT, 10, 0, ALT, 0), (LAND, 10, 0, 0, 0)]
        t_arm = run_cycle(res, st, mission)
        g = st.gcs
        ev = phase_events(st)
        seq = [p for _, p, _ in ev]
        res.metric("phase_sequence", seq)
        clk = st.module_clock(1)
        t_to = next((t for t, p, _ in ev if p == "TAKEOFF"), None)
        t_fl = next((t for t, p, _ in ev if p == "FLIGHT"), None)
        stuck = [clk(r["t"]) for r in module_texts(st) if r["text"] == "TOF RF STUCK LOW"]
        res.metric("stuck_low_text_after_takeoff_s", stuck and t_to and round(stuck[0] - t_to, 2), key=True)
        res.metric("flight_after_takeoff_s", t_fl and t_to and round(t_fl - t_to, 2), key=True)
        res.check("MAP-15c: 'TOF RF STUCK LOW' after RF_LOW_MAX_TAKEOFF_S in TAKEOFF",
                  bool(stuck) and t_to is not None and 5.0 <= stuck[0] - t_to <= 6.0,
                  stuck and t_to and round(stuck[0] - t_to, 2), "5..6 s")
        res.check("MAP-15c -> MAP-15b: FLIGHT entered once stuck", t_fl is not None and 5.0 <= t_fl - t_to <= 6.5,
                  t_fl and t_to and round(t_fl - t_to, 2), "5..6.5 s after TAKEOFF")
        res.check("MAP-16: cycle ends on the ground", seq[-1:] == ["GROUND"], seq[-1:], "GROUND")
        gcs_txt = [x for x in g.texts(t_arm, H.MODULE_COMPID) if x[3] == "TOF RF STUCK LOW"]
        res.check("'TOF RF STUCK LOW' reaches the GCS", bool(gcs_txt), len(gcs_txt), ">= 1")
        gh = ghost_exports(st, phases=["TAKEOFF", "FLIGHT", "LANDING"])
        res.metric("ground_export_sector_maps_by_phase (MAP-15b: classification off)",
                   {p: sum(1 for x in gh if x["phase"] == p) for p in ("TAKEOFF", "FLIGHT", "LANDING")}, key=True)
        res.note("Ground exported near touchdown is the specified MAP-15b behaviour with a stuck rangefinder.")
    finally:
        st.stop()
    res.stand(st)
    assert res.save() == "PASS", [c for c in res.d["checks"] if not c["ok"]]
