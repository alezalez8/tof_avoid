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

    def metric(self, name, value):
        self.d["metrics"][name] = value

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
    """Largest lateral distance (m) of the track from the straight segment a -> b (n, e)."""
    an, ae = a
    bn, be = b
    ln = math.hypot(bn - an, be - ae)
    worst = 0.0
    for r in scene_recs:
        if r.get("k") != "pose" or r["t"] < t_from or r["h"] < 0.5:
            continue
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
