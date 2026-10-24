"""T-15 (SITL): hold command on a module fault in AUTO / GUIDED / RTL; FC refusing the change; AUTO again by the
pilot (FLT-03, FLT-03a, FLT-04, FLT-06).

Criterion (TZ 13): mode changed <= 1.5 s; on refusal "HOLD REJECTED"; after AUTO again by the pilot no commands.
Fault: all three sensors off (classes MAP LOST + STALE of FAULT_TRIGGER_MASK). FAULT_ACTION = BRAKE (default).
Refusal: FLTMODE_GCSBLOCK bit 12 (BRAKE) - ArduCopter rejects BRAKE from MAVLink.
"""

import time

import pytest

from common import H, Result, first_mode_after, module_hold_commands, sensors


def start_flight(st, kind):
    g = st.gcs
    g.rc_override(c3=1500)
    if kind == "GUIDED":
        g.set_mode("GUIDED")
        time.sleep(1)
        g.arm()
        g.takeoff(3.0)
        g.goto_ned(20, 0, 3.0)
        time.sleep(3)
    else:
        g.upload_mission([(22, 0, 0, 3, 0), (16, 20, 0, 3, 0), (16, 20, 6, 3, 0), (21, 20, 6, 0, 0)])
        g.set_mode("GUIDED")
        time.sleep(1)
        g.arm()
        g.set_mode("AUTO")
        g.wait(lambda: g.pos() and g.pos()[0] > 3.0, 60, "first leg")
        if kind == "RTL":
            g.wait(lambda: g.pos() and g.pos()[0] > 10.0, 60, "outbound")
            g.set_mode("RTL")
            time.sleep(3)


def hold_metrics(res, st, t_off, expect_mode):
    clk = st.module_clock(1)
    cmds = [(clk(r["t"]), r) for r in module_hold_commands(st) if clk(r["t"]) > t_off]
    t_det = next((clk(r["t"]) for r in st.module.records(1) if r.get("k") == "ev" and r["kind"] == 6 and r["b"] == 1
                  and clk(r["t"]) > t_off), None)
    res.metric("hold_commands", len(cmds), key=True)
    res.metric("command_after_detection_s", cmds and t_det and round(cmds[0][0] - t_det, 3), key=True)
    res.check("FLT-03: DO_SET_MODE <= 100 ms after detection", bool(cmds) and t_det is not None and
              cmds[0][0] - t_det <= 0.1, cmds and t_det and round(cmds[0][0] - t_det, 3), "<= 0.1 s")
    if cmds:
        c = cmds[0][1]
        res.check("FLT-03: DO_SET_MODE param1 = 1, param2 = %s" % expect_mode, c["p1"] == 1 and c["p2"] == 17,
                  "p1 %s p2 %s" % (c["p1"], c["p2"]), "1 / 17")
    return cmds, t_det


@pytest.mark.parametrize("kind", ["AUTO", "GUIDED", "RTL"])
def test_T15_hold(kind):
    sub = {"AUTO": "a", "GUIDED": "b", "RTL": "c"}[kind]
    res = Result("T-15", sub, "Команда удержания при отказе модуля в %s%s" % (
        kind, "; повторный AUTO пилотом" if kind == "AUTO" else ""),
        "Режим сменён ≤ 1,5 с; успех — по HEARTBEAT FC" + ("; после возврата AUTO пилотом команд нет" if kind == "AUTO"
                                                          else ""))
    st = H.Stand("T-15_" + sub, scene={"objects": []})
    try:
        st.start()
        g = st.gcs
        start_flight(st, kind)
        res.step("Полёт в %s; выключение всех датчиков" % kind)
        t_off = time.time()
        sensors(st, False)
        time.sleep(4)
        cmds, t_det = hold_metrics(res, st, t_off, "BRAKE")
        t_brake = first_mode_after(g, "BRAKE", t_off)
        res.metric("mode_brake_after_fault_s", t_brake and round(t_brake - t_off, 2), key=True)
        res.check("T-15: FC in BRAKE <= 1.5 s after the fault", t_brake is not None and t_brake - t_off <= 1.5,
                  t_brake and round(t_brake - t_off, 2), "<= 1.5 s")
        res.check("FLT-04: one successful change (no repeats after the HEARTBEAT shows BRAKE)", len(cmds) == 1,
                  len(cmds), 1)
        clk = st.module_clock(1)
        acks = [r for r in st.module.records(1) if r.get("k") == "ev" and r["kind"] == 3 and clk(r["t"]) > t_off]
        res.metric("command_ack_logged", [(r["a"], r["b"]) for r in acks])
        if kind == "AUTO":
            res.step("Пилот возвращает AUTO при сохраняющемся отказе")
            t_re = time.time()
            g.set_mode("AUTO")
            time.sleep(6)
            after = [r for r in module_hold_commands(st) if clk(r["t"]) > t_re]
            res.check("FLT-04: no command after AUTO again by the pilot", not after, len(after), 0)
            res.check("FLT-04: FC stays in AUTO", g.mode() == "AUTO", g.mode(), "AUTO")
            txt = [x for x in g.texts(t_re, H.MODULE_COMPID) if x[3].startswith("TOF FAULT ACTIVE")]
            res.check("FLT-04: warning only ('TOF FAULT ACTIVE, AUTO WITHOUT HOLD' at the GCS)", bool(txt),
                      txt[0][3] if txt else None, "text")
        sent = [x for x in g.texts(t_off, H.MODULE_COMPID) if x[3].startswith("TOF HOLD SENT")]
        res.check("DIA-03: 'TOF HOLD SENT BRAKE' at the GCS", bool(sent), sent[0][3] if sent else None, "text")
        g.set_mode("LAND", wait=False)
        g.wait(lambda: not g.armed(), 60, "disarm")
    finally:
        st.stop()
    res.stand(st)
    assert res.save() == "PASS", [c for c in res.d["checks"] if not c["ok"]]


def test_T15_d_rejected():
    res = Result("T-15", "d", "FC отказывает в переходе в BRAKE (FLTMODE_GCSBLOCK бит BRAKE), AUTO",
                 "При отказе — «TOF HOLD REJECTED»; не более 6 передач с интервалом 500 мс (FLT-03a)")
    st = H.Stand("T-15_d", scene={"objects": []}, fc_params={"FLTMODE_GCSBLOCK": 4096})
    try:
        st.start()
        g = st.gcs
        start_flight(st, "AUTO")
        res.step("Полёт в AUTO; выключение всех датчиков; BRAKE заблокирован для GCS")
        t_off = time.time()
        sensors(st, False)
        time.sleep(6)
        cmds, t_det = hold_metrics(res, st, t_off, "BRAKE")
        ts = [t for t, _ in cmds]
        gaps = [round(b - a, 2) for a, b in zip(ts, ts[1:])]
        res.metric("command_intervals_s", gaps)
        res.check("FLT-03a: 6 transmissions in total", len(cmds) == 6, len(cmds), 6)
        res.check("FLT-03a: interval 500 ms", gaps and all(0.45 <= x <= 0.6 for x in gaps), gaps, "0.5 s")
        res.check("FC stays in AUTO", g.mode() == "AUTO", g.mode(), "AUTO")
        rej = [x for x in g.texts(t_off, H.MODULE_COMPID) if x[3] == "TOF HOLD REJECTED"]
        res.metric("rejected_after_fault_s", rej and round(rej[0][0] - t_off, 2), key=True)
        res.check("FLT-06: 'TOF HOLD REJECTED' at the GCS", bool(rej), rej and round(rej[0][0] - t_off, 2), "text")
        clk = st.module_clock(1)
        acks = [r for r in st.module.records(1) if r.get("k") == "ev" and r["kind"] == 3 and clk(r["t"]) > t_off]
        res.metric("command_ack_results", [r["a"] for r in acks])
        res.check("FLT-03: COMMAND_ACK logged, not taken as success", acks and g.mode() == "AUTO",
                  [r["a"] for r in acks], "logged")
        g.set_mode("LAND", wait=False)
        g.wait(lambda: not g.armed(), 60, "disarm")
    finally:
        st.stop()
    res.stand(st)
    assert res.save() == "PASS", [c for c in res.d["checks"] if not c["ok"]]
