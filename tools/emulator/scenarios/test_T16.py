"""T-16 (SITL): manual modes with an obstacle ahead (FLT-05, FC-02).

Criterion (TZ 13): warnings present; no change of control and no commands. Loiter, PosHold, AltHold, Stabilize,
Acro in turn with a box ahead; pitch stick forward in Loiter / PosHold / AltHold / Stabilize (Acro: sticks neutral);
in Loiter also a module fault (sensors off) - the module must not command (FLT-05).
"""

import time

from common import H, Result, mode_changes, module_commands, sensors

MODES = ["LOITER", "POSHOLD", "ALT_HOLD", "STABILIZE", "ACRO"]


def test_T16_manual_modes():
    res = Result("T-16", "", "Ручные режимы: Loiter, PosHold, AltHold, Stabilize, Acro; препятствие спереди",
                 "Предупреждения есть; изменения управления и команд нет")
    scene = {"objects": [{"type": "box", "id": "box", "n": [3.0, 3.6], "e": [-1.5, 1.5], "h": [0, 8]}]}
    st = H.Stand("T-16", scene=scene)
    windows = []
    try:
        st.start()
        g = st.gcs
        g.rc_override(c1=1500, c2=1500, c3=1500, c4=1500)
        g.set_mode("GUIDED")
        time.sleep(1)
        g.arm()
        g.takeoff(3.0)
        for mode in MODES:
            g.goto_ned(0, 0, 3.0, yaw=0.0)
            time.sleep(7)
            p0 = g.pos()
            g.set_mode(mode)
            t0 = time.time()
            if mode != "ACRO":
                g.rc_override(c2=1400)  # nose down: forward
            time.sleep(2.5)
            g.rc_override(c2=1500)
            time.sleep(1.0)
            if mode == "LOITER":
                res.step("Loiter: отказ модуля (все датчики выключены) на 3 с")
                sensors(st, False)
                time.sleep(3.0)
                sensors(st, True)
                time.sleep(2.0)
            p1 = g.pos()
            t1 = time.time()
            windows.append((mode, t0, t1, p0, p1))
            res.step("%s: %.1f с, ручка тангажа %s" % (mode, t1 - t0, "вперёд" if mode != "ACRO" else "нейтраль"))
            g.set_mode("GUIDED")
        g.goto_ned(0, 0, 3.0)
        time.sleep(4)
        g.set_mode("LAND", wait=False)
        g.wait(lambda: not g.armed(), 60, "disarm")
    finally:
        st.stop()
    cmds = module_commands(st)
    holds = [r for r in cmds if r["cmd"] == 176]
    other = [r["cmd"] for r in cmds if r["cmd"] not in (176, 511)]
    res.check("FLT-05: no DO_SET_MODE from the module in manual modes", not holds, len(holds), 0)
    res.check("FLT-05: no commands other than SET_MESSAGE_INTERVAL", not other, other, "none")
    for mode, t0, t1, p0, p1 in windows:
        modes = [m for t, m in mode_changes(g, t0 - 0.5) if t <= t1]
        res.check("%s: FC mode unchanged in the window" % mode, modes == [mode], modes, [mode])
        warn = [x for x in g.texts(t0 - 3.0, H.MODULE_COMPID) if x[0] <= t1 and x[3].startswith("TOF OBST")]
        res.metric("%s_obstacle_texts" % mode, [x[3] for x in warn])
        state_warn = [x for x in g.texts(0, H.MODULE_COMPID) if x[0] <= t1 and x[3].startswith("TOF OBST")]
        active = state_warn and not state_warn[-1][3].startswith("TOF OBST CLEAR")
        res.check("%s: obstacle warning active at the GCS" % mode, bool(active),
                  state_warn[-1][3] if state_warn else None, "WARN/CRIT")
        if p0 and p1:
            dn = p1[0] - p0[0]
            res.metric("%s_forward_m" % mode, round(dn, 2), key=mode in ("LOITER", "POSHOLD"))
            if mode in ("LOITER", "POSHOLD"):
                res.check("FC-02: %s moves on towards the obstacle (no braking by avoidance)" % mode,
                          p1[0] > 2.5 - H.VEHICLE_RADIUS_M, round(p1[0], 2), "> %.2f m (box at 3.0 m)" %
                          (2.5 - H.VEHICLE_RADIUS_M))
    res.stand(st)
    assert res.save() == "PASS", [c for c in res.d["checks"] if not c["ok"]]
