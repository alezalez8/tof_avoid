"""T-14 (SITL): map LOST (FLT-01, FLT-02).

Criterion (TZ 13): OBSTACLE_DISTANCE stops; PRX NoData <= 1 s. Also FLT-01 detection <= 300 ms and HEARTBEAT
CRITICAL (FLT-02). Hover in GUIDED at 3 m, all three sensors switched off at once.
"""

import time

from common import H, Result, fc_clock, module_heartbeats, sensors


def test_T14_map_lost():
    res = Result("T-14", "", "Карта LOST: все три датчика выключены в висении (GUIDED, 3 м)",
                 "Отправка OBSTACLE_DISTANCE прекращена; PRX NoData ≤ 1 с; обнаружение ≤ 300 мс; HEARTBEAT CRITICAL")
    st = H.Stand("T-14", scene={"objects": [{"type": "box", "id": "box", "n": [2.5, 3.1], "e": [-0.6, 0.6],
                                             "h": [0, 6]}]})
    try:
        st.start()
        g = st.gcs
        g.rc_override(c3=1500)
        g.set_mode("GUIDED")
        time.sleep(1)
        g.arm()
        g.takeoff(3.0)
        time.sleep(5)
        res.step("Висение 3 м, препятствие 2,5 м впереди; выключение всех датчиков")
        t_off = time.time()
        sensors(st, False)
        time.sleep(6)
        g.set_mode("LAND", wait=False)
        g.wait(lambda: not g.armed(), 60, "disarm")
    finally:
        st.stop()
    clk = st.module_clock(1)
    recs = st.module.records(1)
    lost = [clk(r["t3"]) for r in recs if r.get("k") == "map" and r["ms"] == 2 and clk(r["t3"]) > t_off]
    t_lost = lost[0] if lost else None
    flt = [clk(r["t"]) for r in recs if r.get("k") == "ev" and r["kind"] == 6 and r["b"] == 1 and clk(r["t"]) > t_off]
    t_det = flt[0] if flt else None
    last_frame = max((clk(r["t1"]) for r in recs if r.get("k") == "map" and r["ms"] != 2 and clk(r["t1"]) <= t_off + 0.2),
                     default=None)
    res.metric("detection_after_switch_off_s", t_det and round(t_det - t_off, 3), key=True)
    res.metric("detection_after_last_frame_s", t_det and last_frame and round(t_det - last_frame, 3))
    res.check("FLT-01: fault detected <= 300 ms after the last frame", t_det and last_frame and
              t_det - last_frame <= 0.30 + 0.02, t_det and last_frame and round(t_det - last_frame, 3), "<= 0.30 s")
    res.metric("map_lost_after_switch_off_s", t_lost and round(t_lost - t_off, 3), key=True)
    od = [t for t, m in g.history("OBSTACLE_DISTANCE", H.MODULE_COMPID, t_off - 1)]
    after = [t for t in od if t_lost and t > t_lost + 0.2]
    t_last_od = max([t for t in od if not t_lost or t <= t_lost + 0.2], default=None)
    res.metric("od_after_lost", len(after))
    res.check("FLT-02: OBSTACLE_DISTANCE stopped at LOST", t_lost is not None and not after, len(after), 0)
    sys = [(t, bool(m.onboard_control_sensors_health & H.PRX_BIT)) for t, m in g.history("SYS_STATUS", 1, t_off - 1)]
    t_nd = next((t for t, ok in sys if not ok and t_last_od and t > t_last_od), None)
    res.metric("prx_nodata_after_last_od_s (SYS_STATUS 5 Hz)", t_nd and round(t_nd - t_last_od, 2), key=True)
    res.metric("prx_nodata_after_lost_s", t_nd and t_lost and round(t_nd - t_lost, 2), key=True)
    res.check("T-14: PRX NoData <= 1 s after LOST", t_nd is not None and t_nd - t_lost <= 1.0,
              t_nd and round(t_nd - t_lost, 2), "<= 1.0 s")
    fcc = fc_clock(g)
    prx = [(fcc(x.TimeUS / 1e6), x.He) for x in H.bin_messages(st.sitl.bin_logs()[-1], ["PRX"])]
    t_he = next((t for t, he in prx if he == 1 and t > t_off), None)
    res.metric("prx_log_nodata_after_lost_s", t_he and t_lost and round(t_he - t_lost, 2))
    hb = module_heartbeats(g, t_off - 1)
    crit = [t for t, s in hb if s == 5 and t_lost and t > t_lost]
    res.check("FLT-02: module HEARTBEAT CRITICAL while LOST", bool(crit), len(crit), ">= 1")
    res.check("FLT-02: HEARTBEAT continues while LOST", len([t for t, s in hb if t_lost and t > t_lost]) >= 4,
              len([t for t, s in hb if t_lost and t > t_lost]), ">= 4 in 6 s")
    res.stand(st)
    assert res.save() == "PASS", [c for c in res.d["checks"] if not c["ok"]]
