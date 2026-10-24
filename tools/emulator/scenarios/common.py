"""Shared scenario helpers: results for the reports and run_all.sh, log analysis."""

import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import harness as H  # noqa: E402

RESULTS = H.RUN_ROOT / "results"
FC_VERSION = "ArduCopter V4.7.2 (Copter-4.7.2, fb773567)"


class Result:
    """Checks of one scenario; saved as build/sitl_run/results/<id>.json."""

    def __init__(self, test, sub, title, criterion, strict=True):
        self.d = {"test": test, "sub": sub, "title": title, "criterion": criterion, "strict": strict,
                  "checks": [], "metrics": {}, "notes": [], "steps": [], "date": time.strftime("%Y-%m-%d"),
                  "t_start": time.time()}

    @property
    def id(self):
        return "%s%s" % (self.d["test"], ("_" + self.d["sub"]) if self.d["sub"] else "")

    def step(self, text):
        self.d["steps"].append(text)
        print("[%s] %s" % (self.id, text), flush=True)

    def check(self, name, ok, value=None, limit=None):
        self.d["checks"].append({"name": name, "ok": bool(ok), "value": value, "limit": limit})
        print("[%s] %s %s: %s (limit %s)" % (self.id, "OK  " if ok else "FAIL", name, value, limit), flush=True)
        return ok

    def metric(self, name, value, key=False):
        """key: shown in the run_all.sh summary table."""
        self.d["metrics"][name] = value
        if key:
            self.d.setdefault("key", []).append(name)

    def note(self, text):
        self.d["notes"].append(text)

    def stand(self, st):
        """Configuration of the run: module version and parameter CRC, FC parameter files CRC."""
        start = [r for r in st.module.records(1) if r.get("k") == "start"]
        if start:
            s = start[0]
            self.d["module"] = {"ver": s["ver"], "git": s["git"], "params_crc": s["params_crc"], "load": s["load"],
                                "params_set": st.module.params}
        self.d["fc"] = {"ver": FC_VERSION, "params_crc": H.param_file_crc(st.sitl.param_files),
                        "params_extra": st.sitl.params}
        self.d["run_dir"] = str(st.dir.relative_to(H.ROOT))
        self.d["scene"] = st.scene.config
        # Bench real time: the scene must deliver frames without gaps (a gap > 3 periods is a sensor fault for the
        # module, SNS-11) - otherwise the run says nothing about the module.
        ts = [r["t"] for r in st.scene.records() if r.get("k") == "frame" and r.get("s") == 1]
        gap = max((b - a for a, b in zip(ts, ts[1:])), default=0.0)
        self.metric("bench_scene_max_frame_gap_s", round(gap, 3))
        self.check("bench: scene in real time (max frame gap <= 0.2 s)", gap <= 0.2, round(gap, 3), "<= 0.2 s")

    @property
    def passed(self):
        return all(c["ok"] for c in self.d["checks"])

    def save(self):
        RESULTS.mkdir(parents=True, exist_ok=True)
        self.d["t_end"] = time.time()
        self.d["result"] = ("PASS" if self.passed else "FAIL") if self.d["strict"] else \
            ("INFO" if self.passed else "INFO-FAIL")
        (RESULTS / (self.id + ".json")).write_text(json.dumps(self.d, indent=1, ensure_ascii=False, default=str))
        return self.d["result"]


# ---------------------------------------------------------------- analysis

def collisions(scene_recs, radius=H.VEHICLE_RADIUS_M, t_from=0.0):
    """Minimum clearance (m) to scene objects over the pose log and the collision intervals."""
    poses = [r for r in scene_recs if r.get("k") == "pose" and r["t"] >= t_from and r.get("clr") is not None]
    if not poses:
        return float("inf"), [], None
    best = min(poses, key=lambda r: r["clr"])
    hits = [r for r in poses if r["clr"] < radius]
    return best["clr"], hits, best


def path_deviation(scene_recs, a, b, t_from=0.0):
    """Largest lateral distance (m) of the track from the segment a -> b (n, e), from the last time near a until
    the vehicle first comes within 1 m of b (one leg)."""
    an, ae = a
    bn, be = b
    ln = math.hypot(bn - an, be - ae)
    poses = [r for r in scene_recs if r.get("k") == "pose" and r["t"] >= t_from and r["h"] >= 0.5]
    end = next((i for i, r in enumerate(poses) if math.hypot(r["n"] - bn, r["e"] - be) < 1.0), len(poses))
    start = max((i for i, r in enumerate(poses[:end]) if math.hypot(r["n"] - an, r["e"] - ae) < 1.0), default=0)
    worst = 0.0
    for r in poses[start:end]:
        d = abs((bn - an) * (ae - r["e"]) - (an - r["n"]) * (be - ae)) / ln
        worst = max(worst, d)
    return worst


def module_commands(st):
    """COMMAND_LONG packets the module put on the wire (all boots)."""
    return [r for r in st.module.records() if r.get("k") == "tx" and r.get("id") == 76]


def module_hold_commands(st):
    return [r for r in module_commands(st) if r["cmd"] == 176]


def module_texts(st):
    return [r for r in st.module.records() if r.get("k") == "tx" and r.get("id") == 253]


def mode_changes(gcs, since=0.0):
    """FC mode timeline from HEARTBEAT: list of (t, mode name)."""
    out = []
    for t, m in gcs.fc_heartbeats(since):
        name = H.MODE_NAMES.get(m.custom_mode, str(m.custom_mode))
        if not out or out[-1][1] != name:
            out.append((t, name))
    return out


def oa_db_items(gcs, since=0.0):
    """OA database items seen on the GCS (ADSB_VEHICLE "OA_DB"): (t, n, e, alt_m, radius_m)."""
    out = []
    for t, m in gcs.history("ADSB_VEHICLE", 1, since):
        if not m.callsign.startswith("OA_DB"):
            continue
        n, e = H.latlon_to_ned(m.lat / 1e7, m.lon / 1e7)
        out.append((t, n, e, m.altitude / 1000.0, m.squawk / 100.0, m.ICAO_address))
    return out


def ghost_items(items, scene_cfg, tol=0.6):
    """OA items not explained by a scene object (ground or false points): farther than tol from every object."""
    objs = [H_obj(o) for o in scene_cfg.get("objects", [])]
    out = []
    for it in items:
        if it[4] <= 0.0:  # radius 0: item expired
            continue
        if all(o.clearance(it[1], it[2], o_h(o)) > tol for o in objs):
            out.append(it)
    return out


def H_obj(o):
    import scene
    return scene.make_object(o)


def o_h(o):
    """A height inside the object's span for a horizontal clearance query."""
    return getattr(o, "h0", 0.0) + 0.01 if hasattr(o, "h0") else 0.01


def phase_sequence(st, boot=1):
    ph = [r["ph"] for r in st.module.records(boot) if r.get("k") == "map"]
    seq = []
    for p in ph:
        if not seq or seq[-1] != p:
            seq.append(p)
    return [H.PHASES[p] for p in seq]


def flight_start_mission(st, items, alt):
    """Upload a mission, arm in GUIDED, switch to AUTO with throttle up; returns the AUTO start time."""
    g = st.gcs
    g.upload_mission(items)
    g.rc_override(c3=1500)
    g.set_mode("GUIDED")
    g.arm()
    g.set_mode("AUTO")
    return time.time()


def pose_at(poses, t):
    """Scene pose record nearest in time (poses sorted by t)."""
    import bisect
    ts = [p["t"] for p in poses]
    i = bisect.bisect_left(ts, t)
    cand = [poses[k] for k in (i - 1, i) if 0 <= k < len(poses)]
    return min(cand, key=lambda p: abs(p["t"] - t)) if cand else None


def _hray(objs, n, e, bearing, h=1.0):
    """Horizontal range from (n, e) along a bearing to the nearest object, at the vehicle height h or below it
    (the sensors see objects below the horizon too; exported distances are horizontal)."""
    d = (math.cos(bearing), math.sin(bearing), 0.0)
    best = None
    for hh in sorted({h, 0.3, 1.0, 2.0}):
        if hh > h + 1e-6:
            continue
        for ob in objs:
            t = ob.ray((n, e, -hh), d, 0.0)
            if t is not None and (best is None or t < best):
                best = t
    return best


EXPLAIN_RANGE_M = 0.35
EXPLAIN_SPREAD_DEG = 20.0  # zone width + CAL_TOL_DEG + sector (MAP-05)
EXPLAIN_WINDOW_S = 0.8     # HOLD_MAX_MS + latency


def explained(objs, poses, t, rng, bearing_fn, window=EXPLAIN_WINDOW_S):
    """True if a target at range rng (m) reported at time t is a real object seen from some pose of the last
    `window` seconds (MAP-13 hold while turning): bearing_fn(pose) gives its world bearing (rad) seen from that
    pose."""
    import bisect
    ts = [p["t"] for p in poses]
    i0 = bisect.bisect_left(ts, t - window)
    i1 = bisect.bisect_right(ts, t + 0.1)
    for p in poses[max(i0 - 1, 0):i1]:
        b0 = bearing_fn(p)
        for k in range(-int(EXPLAIN_SPREAD_DEG), int(EXPLAIN_SPREAD_DEG) + 1, 2):
            r = _hray(objs, p["n"], p["e"], b0 + math.radians(k), max(p["h"], 0.3))
            if r is not None and abs(r - rng) <= EXPLAIN_RANGE_M:
                return True
    return False


def ghost_exports(st, phases=None, max_cm=300):
    """Exported obstacle sectors (d <= max) not explained by a scene object (e.g. the ground), see explained()."""
    import scene as S
    import bisect
    objs = [S.make_object(o) for o in st.scene.config.get("objects", [])]
    recs = st.scene.records()
    poses = [r for r in recs if r.get("k") == "pose"]
    # frames in which the scene truth has a ground target within MAX_DIST (+ noise margin): only maps after such
    # frames (MAP-13 hold window) can carry the ground
    g_ts = sorted(r["t"] for r in recs if r.get("k") == "frame" and any(
        len(x) == 2 and x[1] == "ground" and x[0] <= max_cm / 100.0 + 0.3 for tg in r["z"] for x in tg))
    out = []
    for boot in range(1, len(st.module.logs) + 1):
        clk = st.module_clock(boot)
        for r in st.module.records(boot):
            if r.get("k") != "map" or not r["sent"] or (phases and H.PHASES[r["ph"]] not in phases):
                continue
            t = clk(r["t3"])
            p = pose_at(poses, t)
            i = bisect.bisect_right(g_ts, t)
            if p is None or i == 0 or t - g_ts[i - 1] > EXPLAIN_WINDOW_S:
                continue  # no ground in range in the scene: nothing exported can be the ground
            for k, dcm in enumerate(r["d"]):
                if dcm < 4 or dcm > max_cm:
                    continue
                if not explained(objs, poses, t, dcm / 100.0, lambda q, k=k: q["yaw"] + math.radians(k * 5.0)):
                    out.append({"t": t, "phase": H.PHASES[r["ph"]], "sector": k, "cm": dcm, "h": p["h"]})
    return out


OA_REPORT_WINDOW_S = 2.0  # MAP-13 hold + FC reception + ADSB_VEHICLE report backlog


def ghost_oa_items(st, items):
    """OA database items (first report of each index and position) not explained by a scene object.

    The FC places the sectors of a packet with its own attitude at reception (time_usec is not used), so while
    the vehicle turns a real object is smeared in bearing: an item is compared in the body frame (bearing minus the
    yaw at its report) with what the module saw from earlier poses."""
    import scene as S
    objs = [S.make_object(o) for o in st.scene.config.get("objects", [])]
    poses = [r for r in st.scene.records() if r.get("k") == "pose"]
    seen, out = set(), []
    for it in items:
        t, n, e, alt, rad, idx = it
        key = (idx, round(n, 1), round(e, 1))
        if rad <= 0.0 or key in seen:
            continue
        seen.add(key)
        p = pose_at(poses, t)
        if p is None:
            continue
        rng = math.hypot(n - p["n"], e - p["e"])
        body = math.atan2(e - p["e"], n - p["n"]) - p["yaw"]
        if not explained(objs, poses, t, rng, lambda q, body=body: q["yaw"] + body, window=OA_REPORT_WINDOW_S):
            out.append({"t": t, "n": round(n, 2), "e": round(e, 2), "range": round(rng, 2),
                        "body_bearing_deg": round(math.degrees(body), 1), "idx": idx})
    return out


def ground_targets(st, t0, t1, max_m=3.0):
    """Zones of the scene truth with a ground target closer than max_m between t0 and t1 (data the module
    excludes as GROUND_SUSPECT in ground and transition phases)."""
    n = 0
    for r in st.scene.records():
        if r.get("k") == "frame" and t0 <= r["t"] <= t1:
            for tg in r["z"]:
                if any(len(x) == 2 and x[1] == "ground" and x[0] <= max_m for x in tg):
                    n += 1
    return n


def phase_events(st, boot=1):
    """Phase changes logged by the module: list of (wall time, new phase, previous phase)."""
    clk = st.module_clock(boot)
    return [(clk(r["t"]), H.PHASES[r["a"]], H.PHASES[r["b"]]) for r in st.module.records(boot)
            if r.get("k") == "ev" and r["kind"] == 5]


def landed_states(gcs, since=0.0):
    out = []
    for t, m in gcs.history("EXTENDED_SYS_STATE", 1, since):
        if not out or out[-1][1] != m.landed_state:
            out.append((t, m.landed_state))
    return out


def oa_active(st):
    """FC log: BendyRuler activity (OABR.Act) as a list of (TimeUS s, active)."""
    logs = st.sitl.bin_logs()
    if not logs:
        return []
    return [(x.TimeUS / 1e6, x.Act) for x in H.bin_messages(logs[-1], ["OABR"])]


def fc_log_modes(st):
    logs = st.sitl.bin_logs()
    return [(x.TimeUS / 1e6, x.Mode) for x in H.bin_messages(logs[-1], ["MODE"])] if logs else []


# ---------------------------------------------------------------- flights with injections

class Inject:
    """A runtime event: when `when(stand, t_rel)` becomes true, `do(stand)` runs once."""

    def __init__(self, name, when, do):
        self.name, self.when, self.do, self.t = name, when, do, None


def at_time(sec):
    return lambda st, t: t >= sec


def near(n, e, dist):
    def f(st, t):
        p = st.gcs.pos()
        return p is not None and math.hypot(p[0] - n, p[1] - e) <= dist
    return f


def sensors(st, powered, which=(0, 1, 2)):
    for k in which:
        st.scene.cmd(cmd="sensor", sensor=k, powered=powered)


def apply_injections(st, injects, t0, res):
    now = time.time()
    for inj in injects:
        if inj.t is None and inj.when(st, now - t0):
            inj.t = now
            inj.do(st)
            res.step("t+%.1f с: %s" % (now - t0, inj.name))


def fly(st, res, kind, target, injects=(), timeout=120.0, alt=3.0, mission=None, tol=1.0):
    """kind: "AUTO" (mission, ends with LAND), "GUIDED" (takeoff + goto target), "RTL" (mission to target, then RTL
    home). Returns (t0 of the horizontal phase, reached target flag, time to reach)."""
    g = st.gcs
    g.rc_override(c3=1500)
    tn, te = target
    if kind == "AUTO":
        g.upload_mission(mission or [(22, 0, 0, alt, 0), (16, tn, te, alt, 0), (21, tn, te, 0, 0)])
        g.set_mode("GUIDED")
        time.sleep(1)
        g.arm()
        g.set_mode("AUTO")
        g.wait(lambda: g.pos() and -g.pos()[2] > alt - 0.4, 40, "takeoff")
    elif kind == "GUIDED":
        g.set_mode("GUIDED")
        time.sleep(1)
        g.arm()
        g.takeoff(alt)
        time.sleep(2)
        g.goto_ned(tn, te, alt)
    elif kind == "RTL":
        g.upload_mission(mission or [(22, 0, 0, alt, 0), (16, tn, te, alt, 0), (17, tn, te, alt, 0)])
        g.set_mode("GUIDED")
        time.sleep(1)
        g.arm()
        g.set_mode("AUTO")
        g.wait(lambda: g.pos() and math.hypot(g.pos()[0] - tn, g.pos()[1] - te) < 1.0, 90, "outbound leg")
        time.sleep(2)
        g.set_mode("RTL")
        tn, te = 0.0, 0.0
    t0 = time.time()
    res.step("%s: цель (%.1f, %.1f), высота %.1f м" % (kind, tn, te, alt))
    t_reach = None
    while time.time() - t0 < timeout:
        apply_injections(st, injects, t0, res)
        p = g.pos()
        if p and math.hypot(p[0] - tn, p[1] - te) < tol and t_reach is None:
            t_reach = time.time() - t0
            if all(i.t is not None for i in injects):
                break
        if kind == "AUTO" and not g.armed():
            break
        time.sleep(0.1)
    return t0, t_reach is not None, t_reach


def land_and_disarm(st, timeout=60.0):
    g = st.gcs
    if g.armed():
        if g.mode() not in ("LAND",):
            g.set_mode("LAND", wait=False)
        try:
            g.wait(lambda: not g.armed(), timeout, "disarm")
        except TimeoutError:
            g.disarm(force=True)


def avoid_metrics(res, st, t0, strict_collision):
    """Clearance and collision (scene truth), BendyRuler activity, false activations far from objects (FC-06)."""
    sc = st.scene.records()
    clr, hits, best = collisions(sc, t_from=t0)
    res.metric("min_clearance_m", None if clr == float("inf") else round(clr, 2), key=True)
    if best:
        res.metric("min_clearance_at", {k: best[k] for k in ("n", "e", "h", "obj")})
    res.metric("collision_s", round(len(hits) * 0.1, 1), key=True)
    if strict_collision:
        res.check("no collision (clearance > vehicle radius %.2f m)" % H.VEHICLE_RADIUS_M, not hits,
                  None if clr == float("inf") else round(clr, 2), "> %.2f m" % H.VEHICLE_RADIUS_M)
    act = [(x.TimeUS / 1e6, x.Act) for x in H.bin_messages(st.sitl.bin_logs()[-1], ["OABR"])] if \
        st.sitl.bin_logs() else []
    res.metric("oabr_active_records", sum(1 for _, a in act if a), key=True)
    # FC-06: BendyRuler active although no object is within lookahead + margin (4 m) of the vehicle
    off = [t - m.time_boot_ms / 1000.0 for t, m in st.gcs.history("ATTITUDE", 1)]
    o = sorted(off)[len(off) // 2] if off else 0.0
    poses = [r for r in sc if r.get("k") == "pose"]
    false_act = 0
    for t, a in act:
        if not a:
            continue
        p = pose_at(poses, t + o)
        if p and (p.get("clr") is None or p["clr"] > 4.0):
            false_act += 1
    res.metric("fc06_bendyruler_active_without_object_within_4m", false_act, key=True)
    return clr, hits


def fc_clock(g):
    """FC boot time (s) -> wall clock, from ATTITUDE time_boot_ms seen by the GCS."""
    off = [t - m.time_boot_ms / 1000.0 for t, m in g.history("ATTITUDE", 1)]
    o = sorted(off)[len(off) // 2] if off else 0.0
    return lambda s: s + o


def module_heartbeats(g, since=0.0):
    return [(t, m.system_status) for t, m in g.history("HEARTBEAT", H.MODULE_COMPID, since)]


def first_mode_after(g, name, since):
    return next((t for t, m in mode_changes(g, since) if m == name), None)
