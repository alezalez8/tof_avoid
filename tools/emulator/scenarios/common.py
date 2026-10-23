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
    g.rc_override(c3=1500, c7=2000)
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
    """Horizontal range from (n, e) at height h (m above the ground) along a bearing to the nearest object."""
    o = (n, e, -h)
    d = (math.cos(bearing), math.sin(bearing), 0.0)
    best = None
    for ob in objs:
        t = ob.ray(o, d, 0.0)
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
    objs = [S.make_object(o) for o in st.scene.config.get("objects", [])]
    poses = [r for r in st.scene.records() if r.get("k") == "pose"]
    out = []
    for boot in range(1, len(st.module.logs) + 1):
        clk = st.module_clock(boot)
        for r in st.module.records(boot):
            if r.get("k") != "map" or not r["sent"] or (phases and H.PHASES[r["ph"]] not in phases):
                continue
            t = clk(r["t3"])
            p = pose_at(poses, t)
            if p is None:
                continue
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
