"""T-19 (SITL): STATUSTEXT of the module up to the GCS (DIA-03, DIA-03a, DIA-04).

Criterion (TZ 13): every DIA-03 event is seen at the GCS; rates kept (<= 1 message/s per event type, <= 3/s in
total, only on a change of state). a) one run that walks through the events: start with blank parameter flash,
obstacle WARN / CRIT / CLEAR and a flapping obstacle at the WARN threshold, sensor L lost and restored, data stale
and map lost, FC telemetry lost (module RX cut), hold sent and rejected in AUTO, module restart in flight;
b) no rangefinder on the ground (DIA-03a "TOF RF INVALID ON GROUND").
"""

import re
import time

from common import H, Result, sensors

import sil_proto  # noqa: E402  (path set by common)

EXPECTED_A = ["TOF PARAMS DEFAULT", "TOF ORIENT NOT CONFIRMED", "TOF READY", "TOF OBST WARN", "TOF OBST CRIT",
              "TOF OBST CLEAR", "TOF SENSOR L LOST", "TOF SENSOR L OK", "TOF DATA STALE", "TOF MAP LOST",
              "TOF FC LINK LOST", "TOF PHASE UNKNOWN", "TOF HOLD SENT BRAKE", "TOF HOLD REJECTED",
              "TOF REBOOTED IN FLIGHT"]


def ahead(oid, d, w=1.2):
    return {"type": "box", "id": oid, "n": [d, d + 0.4], "e": [-w / 2, w / 2], "h": [0, 6]}


def text_type(t):
    """Event type of a text for the per-type rate: digits and the obstacle numbers removed."""
    return re.sub(r" [-0-9.]+m [-0-9]+deg$", "", re.sub(r"TOF READY .*", "TOF READY", t))


def rate_checks(res, texts):
    ts = [t for t, _ in texts]
    worst = 0
    for i, t in enumerate(ts):
        worst = max(worst, sum(1 for u in ts[i:] if u - t < 1.0))
    res.metric("max_texts_in_1s", worst, key=True)
    res.check("DIA-04: <= 3 module messages in any 1 s at the GCS", worst <= 3, worst, "<= 3")
    by = {}
    for t, x in texts:
        by.setdefault(text_type(x), []).append(t)
    gaps = {k: round(min(b - a for a, b in zip(v, v[1:])), 2) for k, v in by.items() if len(v) > 1}
    res.metric("min_gap_same_type_s", gaps)
    bad = {k: g for k, g in gaps.items() if g < 0.95}
    res.check("DIA-04: <= 1 message/s per event type", not bad, bad, ">= 1 s")
    rep = [x for (_, x), (_, y) in zip(texts, texts[1:]) if x == y and not x.startswith("TOF READY")]
    res.metric("immediate_repeats", rep)


def test_T19_a_events():
    res = Result("T-19", "a", "STATUSTEXT модуля до GCS: события DIA-03 и правила DIA-04",
                 "Все события DIA-03 видны на GCS; частоты соблюдены")
    st = H.Stand("T-19_a", scene={"objects": []}, blank_flash=True, fc_params={"FLTMODE_GCSBLOCK": 4096})
    try:
        st.start()
        g = st.gcs
        sc = st.scene
        res.step("Старт с пустой Flash параметров; препятствие 2,0 м (WARN), 1,0 м (CRIT), убрано (CLEAR)")
        time.sleep(3)
        sc.cmd(cmd="add", obj=ahead("box", 2.0))
        time.sleep(3)
        sc.cmd(cmd="add", obj=ahead("box", 1.0))
        time.sleep(3)
        sc.cmd(cmd="remove", id="box")
        time.sleep(3)
        res.step("Дребезг у порога WARN (2,4 / 2,7 м каждые 0,3 с, 6 с)")
        for k in range(20):
            sc.cmd(cmd="add", obj=ahead("box", 2.4 if k % 2 == 0 else 2.7))
            time.sleep(0.3)
        sc.cmd(cmd="remove", id="box")
        time.sleep(3)
        res.step("Датчик L выключен до FAILED, затем включён")
        sensors(st, False, (0,))
        g.wait(lambda: any(x[3] == "TOF SENSOR L LOST" for x in g.texts(0, H.MODULE_COMPID)), 40, "SENSOR L LOST")
        time.sleep(1)
        sensors(st, True, (0,))
        g.wait(lambda: any(x[3] == "TOF SENSOR L OK" for x in g.texts(0, H.MODULE_COMPID)), 30, "SENSOR L OK")
        time.sleep(2)
        res.step("Все датчики выключены на 2 с (DATA STALE, MAP LOST)")
        sensors(st, False)
        time.sleep(2)
        sensors(st, True)
        time.sleep(6)
        res.step("Обрыв приёма модуля от FC на 4 с (FC LINK LOST, PHASE UNKNOWN)")
        st.module.control(sil_proto.CMD_RX_MUTE, 4000)
        time.sleep(8)
        res.step("AUTO: отказ датчиков при заблокированном BRAKE (HOLD SENT, HOLD REJECTED)")
        g.rc_override(c3=1500)
        g.upload_mission([(22, 0, 0, 3, 0), (16, 30, 0, 3, 0), (21, 30, 0, 0, 0)])
        g.set_mode("GUIDED")
        time.sleep(1)
        g.arm()
        g.set_mode("AUTO")
        g.wait(lambda: g.pos() and g.pos()[0] > 3.0, 60, "first leg")
        sensors(st, False)
        time.sleep(5)
        sensors(st, True)
        time.sleep(4)
        res.step("Перезапуск модуля в полёте (AUTO, armed)")
        st.module.stop(kill=True)
        time.sleep(1)
        st.module.start()
        time.sleep(8)
        g.set_mode("LAND", wait=False)
        g.wait(lambda: not g.armed(), 90, "disarm")
        time.sleep(2)
    finally:
        st.stop()
    texts = [(t, x) for t, c, sev, x in g.texts(0, H.MODULE_COMPID)]
    res.metric("texts", [(round(t - st.t0, 1), x) for t, x in texts])
    seen = {text_type(x) for _, x in texts}
    for e in EXPECTED_A:
        res.check("DIA-03: '%s' at the GCS" % e, e in seen, e in seen, "seen")
    sent = [r["text"] for r in st.module.records() if r.get("k") == "tx" and r.get("id") == 253]
    res.metric("module_sent", len(sent), key=True)
    res.metric("gcs_received", len(texts), key=True)
    res.check("every module STATUSTEXT reaches the GCS", len(texts) == len(sent), "%d / %d" % (len(texts), len(sent)),
              "equal")
    res.check("all texts start with 'TOF' and are <= 50 characters",
              all(x.startswith("TOF") and len(x) <= 50 for _, x in texts), "ok", "TOF, <= 50")
    rate_checks(res, texts)
    flap = [x for t, x in texts if x.startswith("TOF OBST")]
    res.metric("obstacle_texts", flap)
    res.stand(st)
    assert res.save() == "PASS", [c for c in res.d["checks"] if not c["ok"]]


def test_T19_b_rf_invalid_on_ground():
    res = Result("T-19", "b", "Нет дальномера на стоянке (DIA-03a)", "«TOF RF INVALID ON GROUND» на GCS")
    st = H.Stand("T-19_b", scene={"objects": []}, fc_params={"RNGFND1_TYPE": 0})
    try:
        st.start()
        g = st.gcs
        time.sleep(6)
    finally:
        st.stop()
    texts = [x for t, c, sev, x in g.texts(0, H.MODULE_COMPID)]
    res.metric("texts", texts)
    res.check("DIA-03a: 'TOF RF INVALID ON GROUND' at the GCS", "TOF RF INVALID ON GROUND" in texts,
              "TOF RF INVALID ON GROUND" in texts, "seen")
    res.check("sent once (state change only)", texts.count("TOF RF INVALID ON GROUND") == 1,
              texts.count("TOF RF INVALID ON GROUND"), 1)
    res.stand(st)
    assert res.save() == "PASS", [c for c in res.d["checks"] if not c["ok"]]
