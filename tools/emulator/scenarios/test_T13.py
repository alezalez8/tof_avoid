"""T-13 (SITL): BendyRuler in AUTO / GUIDED / RTL with the module map (FC-03, H-08, FC-06).

Criterion (TZ 13): a collision in an "in FoV" scenario is a failure; the behaviour in the blind scenarios goes to
section 16. Cases of the table: obstacle in the FoV (AUTO, GUIDED, RTL), unknown sectors, obstacle at the side
outside the FoV, dead end, lateral offset, turn in place and move, obstacle in a blind sector on the avoidance path,
sensor failure during the avoidance, false target for 1-3 frames, map loss. FC-06 (TZ 5.8): BendyRuler activity
without an object within 4 m (OA database points displaced in fast turns) is counted in every case.
Path: north from (0, 0) to (14, 0) at 3 m, 1 m/s; objects 6 m tall.
"""

import pytest

from common import H, Inject, Result, at_time, avoid_metrics, fly, land_and_disarm, module_hold_commands, near, \
    path_deviation, sensors

BOX = {"type": "box", "id": "box", "n": [6.0, 6.6], "e": [-0.6, 0.6], "h": [0, 6]}


def box(oid, n0, n1, e0, e1, kind="box"):
    return {"type": kind, "id": oid, "n": [n0, n1], "e": [e0, e1], "h": [0, 6]}


def phantom(zones, frames):
    return lambda st: st.scene.cmd(cmd="phantom", sensor=1, zones=zones, dist=1.0, frames=frames)


CASES = [
    # id, title, strict, kind, target, objects, injections, fc params, extra
    ("a", "Препятствие в FoV, AUTO", True, "AUTO", (14, 0), [BOX], [], {}, {}),
    ("b", "Препятствие в FoV, GUIDED", True, "GUIDED", (14, 0), [BOX], [], {}, {}),
    ("c", "Препятствие в FoV, RTL", True, "RTL", (14, 0), [box("box", 7.0, 7.6, -0.6, 0.6)], [],
     {"RTL_ALT_M": 3}, {"mission": [(22, 0, 0, 3, 0), (16, 0, 8, 3, 0), (16, 14, 8, 3, 0), (16, 14, 0, 3, 0),
                                     (17, 14, 0, 3, 0)]}),
    ("d", "Неизвестные сектора: засветка датчика C, препятствие впереди", False, "AUTO", (14, 0), [BOX],
     [Inject("засветка C (все зоны невалидны)", at_time(0.0),
             lambda st: st.scene.cmd(cmd="invalid", sensor=1, frac=1.0))], {}, {}),
    ("e", "Препятствие сбоку вне FoV (стена справа) и препятствие на пути", False, "AUTO", (14, 0),
     [box("box", 6.0, 6.6, -0.9, 0.3), box("wall", 3.0, 12.0, 1.6, 1.8, "wall")], [], {}, {}),
    ("f", "Тупик (П-образные стены)", True, "AUTO", (14, 0),
     [box("front", 9.0, 9.3, -2.0, 2.0, "wall"), box("left", 4.0, 9.3, -2.0, -1.7, "wall"),
      box("right", 4.0, 9.3, 1.7, 2.0, "wall")], [], {}, {"timeout": 70}),
    ("g", "Боковое смещение препятствия (частично на пути)", True, "AUTO", (14, 0),
     [box("box", 6.0, 6.6, 0.4, 1.4)], [], {}, {}),
    ("h", "Поворот на месте и движение (цель на западе, препятствие на пути)", True, "GUIDED", (0, -14),
     [box("box", -0.6, 0.6, -6.6, -6.0)], [], {}, {}),
    ("i", "Препятствие в слепом секторе на пути обхода", False, "AUTO", (14, 0),
     [BOX, box("blind", 4.0, 5.0, 1.8, 2.8), box("blind2", 4.0, 5.0, -2.8, -1.8)], [], {}, {}),
    ("j", "Отказ датчика C во время обхода", False, "AUTO", (14, 0), [BOX],
     [Inject("отказ датчика C (питание)", near(6.0, 0.0, 4.0), lambda st: sensors(st, False, (1,)))], {}, {}),
    ("k", "Ложная цель на 1–3 кадра", False, "AUTO", (14, 0), [],
     [Inject("фантом 1,0 м, 2 кадра", at_time(4.0), phantom([5, 6, 9, 10], 2)),
      Inject("фантом 1,0 м, 1 кадр", at_time(7.0), phantom([5, 6, 9, 10], 1)),
      Inject("фантом 1,0 м, 3 кадра", at_time(10.0), phantom([5, 6, 9, 10], 3))], {}, {}),
    ("l", "Потеря карты в полёте (все датчики)", False, "AUTO", (14, 0), [box("box", 10.0, 10.6, -0.6, 0.6)],
     [Inject("все датчики выключены", at_time(4.0), lambda st: sensors(st, False))], {}, {"timeout": 30}),
]


@pytest.mark.parametrize("case", CASES, ids=[c[0] for c in CASES])
def test_T13(case):
    sub, title, strict, kind, target, objects, injects, fcp, extra = case
    res = Result("T-13", sub, title, "Столкновение в сценарии «в FoV» — провал; поведение в слепых сценариях — "
                 "в раздел 16" if strict else "Слепой/отказный сценарий: факт поведения — в раздел 16", strict=strict)
    st = H.Stand("T-13_" + sub, scene={"objects": objects}, fc_params=fcp)
    try:
        st.start()
        t0, reached, t_reach = fly(st, res, kind, target, injects, timeout=extra.get("timeout", 120),
                                   mission=extra.get("mission"))
        res.metric("target_reached", reached, key=True)
        res.metric("time_to_target_s", t_reach and round(t_reach, 1))
        modes = [m for _, m in __import__("common").mode_changes(st.gcs, t0)]
        res.metric("fc_modes", modes)
        holds = [r for r in module_hold_commands(st)]
        res.metric("module_hold_commands", len(holds), key=bool(holds))
        land_and_disarm(st)
    finally:
        st.stop()
    clr, hits = avoid_metrics(res, st, t0, strict)
    start = (14, 0) if kind == "RTL" else (0, 0)
    res.metric("max_lateral_dev_m", round(path_deviation(st.scene.records(), start, target if kind != "RTL" else
                                                          (0, 0), t0), 2), key=True)
    if not strict:
        res.note("collision: %s; min clearance %.2f m; target reached: %s" % (
            "yes (%.1f s)" % (len(hits) * 0.1) if hits else "no", clr, res.d["metrics"]["target_reached"]))
    res.stand(st)
    r = res.save()
    assert r in ("PASS", "INFO", "INFO-FAIL"), [c for c in res.d["checks"] if not c["ok"]]
