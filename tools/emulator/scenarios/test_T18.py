"""T-18 (SITL): "virtual wall" FAULT_WALL (FLT-10, H-09).

Criterion (TZ 13): a report; until a positive result the function stays off (FAULT_WALL = 0 by default, not
changed here outside the test). Sensor L is switched off in AUTO; once it is FAILED (SNS-11, after 5 failed reinits)
its sectors carry min_distance. FAULT_ACTION = NONE isolates the reaction of the FC to the wall from the hold
command. a) FAULT_WALL = 1; b) baseline FAULT_WALL = 0.
"""

import pytest

from common import H, Result, avoid_metrics, fly, Inject, at_time, land_and_disarm, sensors


@pytest.mark.parametrize("wall", [1, 0])
def test_T18(wall):
    sub = "a" if wall else "b"
    res = Result("T-18", sub, "Отказ датчика L в AUTO, FAULT_WALL = %d, FAULT_ACTION = NONE" % wall,
                 "Отчёт: реакция Copter на «виртуальную стену» (H-09)", strict=False)
    st = H.Stand("T-18_" + sub, scene={"objects": []}, module_params={"FAULT_WALL": wall, "FAULT_ACTION": 0})
    try:
        st.start()
        inj = [Inject("датчик L выключен", at_time(3.0), lambda s: sensors(s, False, (0,)))]
        t0, reached, t_reach = fly(st, res, "AUTO", (40, 0), inj, timeout=80,
                                   mission=[(22, 0, 0, 3, 0), (16, 40, 0, 3, 0), (21, 40, 0, 0, 0)])
        res.metric("target_reached", reached, key=True)
        res.metric("time_to_target_s", t_reach and round(t_reach, 1), key=True)
        land_and_disarm(st)
    finally:
        st.stop()
    clk = st.module_clock(1)
    recs = st.module.records(1)
    t_fail = next((clk(r["t"]) for r in recs if r.get("k") == "st" and r["h"][0] == 2), None)
    walls = [r for r in recs if r.get("k") == "map" and r["sent"] and r["d"].count(4) > 0]
    res.metric("sensor_L_failed_after_off_s", t_fail and round(t_fail - (t0 + 3.0), 1), key=True)
    res.metric("maps_with_wall_sectors", len(walls), key=True)
    if walls:
        res.metric("wall_sectors_deg", [k * 5 if k <= 36 else k * 5 - 360 for k, d in enumerate(walls[-1]["d"]) if d == 4])
    poses = [r for r in st.scene.records() if r.get("k") == "pose" and t_fail and r["t"] > t_fail]
    if len(poses) > 20:
        v = []
        for a, b in zip(poses, poses[10:]):
            v.append(((b["n"] - a["n"]) ** 2 + (b["e"] - a["e"]) ** 2) ** 0.5 / (b["t"] - a["t"]))
        res.metric("speed_after_failed_mps [mean, min]", [round(sum(v) / len(v), 2), round(min(v), 2)], key=True)
    avoid_metrics(res, st, t0, False)
    res.stand(st)
    res.save()
