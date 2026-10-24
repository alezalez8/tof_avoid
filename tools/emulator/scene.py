#!/usr/bin/env python3
"""Synthetic ToF scene for SITL (E3, TZ section 13): frames of three VL53L8CX for tof_sil.

World: local NED of the SITL EKF origin, flat ground at the height of the vehicle on the ground,
boxes, poles (vertical cylinders) and walls (boxes). Sensors: 45 x 45 deg FoV, N x N zones, azimuths
from the module parameters, strictly horizontal (TZ 2), optics 0.20 m above the SITL reference point
(the vehicle sits on it), the module body centre = the optics (MOUNT_XYZ_M = 0). Each zone is sampled
by SUB x SUB rays; hits are clustered by range, up to 2 targets per zone, closest first (SNS-06).
Impairments: range noise, frame drops, glare (share of invalid zones), latency, sensor failure,
phantom targets. The pose comes from SITL (ATTITUDE, LOCAL_POSITION_NED) over its own GCS port and is
taken at mid-exposure.

Run: scene.py --config scene.json  (see SceneConfig); runtime commands as JSON over UDP (--control).
"""

import argparse
import json
import math
import queue
import random
import socket
import sys
import threading
import time

import sil_proto

GROUND_ID = "ground"


# ---------------------------------------------------------------- geometry

def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def mat_mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def mat_vec(m, v):
    return (m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2])


def body_to_ned(roll, pitch, yaw):
    """ZYX: R = Rz(yaw) Ry(pitch) Rx(roll), as ArduPilot and TZ 5.1."""
    return mat_mul(rot_z(yaw), mat_mul(rot_y(pitch), rot_x(roll)))


def dir_sensor(alpha_deg, beta_deg):
    """TZ 5.1: [cos b cos a, cos b sin a, -sin b] (x optical axis, y right, z down)."""
    a, b = math.radians(alpha_deg), math.radians(beta_deg)
    return (math.cos(b) * math.cos(a), math.cos(b) * math.sin(a), -math.sin(b))


def zone_angles(n, i, j):
    """Centre angles of zone (i, j), i = 0 the top row (TZ 5.1)."""
    w = 45.0 / n
    return (j - (n - 1) / 2.0) * w, ((n - 1) / 2.0 - i) * w


class Box:
    """Axis-aligned box in NED: n in [n0, n1], e in [e0, e1], height above ground [h0, h1]."""

    def __init__(self, oid, n0, n1, e0, e1, h1, h0=0.0, kind="box"):
        self.id, self.kind = oid, kind
        self.n0, self.n1 = min(n0, n1), max(n0, n1)
        self.e0, self.e1 = min(e0, e1), max(e0, e1)
        self.h0, self.h1 = h0, h1

    def ray(self, o, d, ground_d):
        """Distance along unit ray d from o to the first hit, or None."""
        lo = (self.n0, self.e0, ground_d - self.h1)
        hi = (self.n1, self.e1, ground_d - self.h0)
        t0, t1 = 0.0, float("inf")
        for k in range(3):
            if abs(d[k]) < 1e-12:
                if o[k] < lo[k] or o[k] > hi[k]:
                    return None
                continue
            a = (lo[k] - o[k]) / d[k]
            b = (hi[k] - o[k]) / d[k]
            if a > b:
                a, b = b, a
            t0, t1 = max(t0, a), min(t1, b)
            if t0 > t1:
                return None
        return t0 if t0 > 1e-6 else None

    def clearance(self, n, e, h):
        """Horizontal distance from a point to the box (0 inside), inf if outside its height span."""
        if h < self.h0 or h > self.h1:
            return float("inf")
        dn = max(self.n0 - n, 0.0, n - self.n1)
        de = max(self.e0 - e, 0.0, e - self.e1)
        return math.hypot(dn, de)

    def to_json(self):
        return {"type": self.kind, "id": self.id, "n": [self.n0, self.n1], "e": [self.e0, self.e1],
                "h": [self.h0, self.h1]}


class Pole:
    """Vertical cylinder: centre (n, e), radius r, height above ground [0, h]."""

    def __init__(self, oid, n, e, r, h):
        self.id, self.kind = oid, "pole"
        self.n, self.e, self.r, self.h = n, e, r, h

    def ray(self, o, d, ground_d):
        dx, dy = o[0] - self.n, o[1] - self.e
        a = d[0] * d[0] + d[1] * d[1]
        if a < 1e-12:
            return None
        b = 2.0 * (dx * d[0] + dy * d[1])
        c = dx * dx + dy * dy - self.r * self.r
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            return None
        sq = math.sqrt(disc)
        for t in ((-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)):
            if t > 1e-6:
                hz = ground_d - (o[2] + t * d[2])  # height above ground of the hit
                if 0.0 <= hz <= self.h:
                    return t
        return None

    def clearance(self, n, e, h):
        if h < 0.0 or h > self.h:
            return float("inf")
        return max(math.hypot(n - self.n, e - self.e) - self.r, 0.0)

    def to_json(self):
        return {"type": "pole", "id": self.id, "c": [self.n, self.e], "r": self.r, "h": self.h}


def make_object(j):
    t = j["type"]
    if t in ("box", "wall"):
        h = j.get("h", [0.0, 3.0])
        return Box(j["id"], j["n"][0], j["n"][1], j["e"][0], j["e"][1], h[1], h[0], kind=t)
    if t == "pole":
        return Pole(j["id"], j["c"][0], j["c"][1], j["r"], j["h"])
    raise ValueError("unknown object type %s" % t)


# ---------------------------------------------------------------- sensor model

class SensorConfig:
    def __init__(self, j=None):
        j = j or {}
        self.azimuths_deg = j.get("azimuths_deg", [-30.0, 0.0, 30.0])  # MOUNT_YAW_DEG_L/C/R
        self.res = j.get("res", 4)
        self.freq_hz = j.get("freq_hz", 15)
        self.sub = j.get("sub", 4)                   # rays per zone side
        self.max_range_m = j.get("max_range_m", 4.0)
        self.min_coverage = j.get("min_coverage", 0.12)
        self.cluster_gap_m = j.get("cluster_gap_m", 0.4)
        self.noise_frac = j.get("noise_frac", 0.01)  # 1-sigma range noise, share of range
        self.noise_min_m = j.get("noise_min_m", 0.005)
        self.optics_h_m = j.get("optics_h_m", 0.20)  # optics above the SITL reference point
        self.exposure_ms = j.get("exposure_ms", None)  # default: one frame period
        self.ambient = j.get("ambient", 2)
        self.seed = j.get("seed", 1)


class SensorState:
    def __init__(self):
        self.powered = True
        self.invalid_frac = 0.0  # glare: share of zones reported with an invalid status
        self.frame_no = 0
        self.phantoms = []       # [zone, distance_m, frames_left]


def precompute_rays(cfg):
    """Body-frame unit rays per sensor, zone (raw ULD index, identity ORIENT_TRANSFORM)."""
    rays = []
    n, sub = cfg.res, cfg.sub
    w = 45.0 / n
    for az in cfg.azimuths_deg:
        rz = rot_z(math.radians(az))
        per_zone = []
        for z in range(n * n):
            i, j = divmod(z, n)
            a_c, b_c = zone_angles(n, i, j)
            zr = []
            for u in range(sub):
                for v in range(sub):
                    a = a_c + ((v + 0.5) / sub - 0.5) * w
                    b = b_c + ((u + 0.5) / sub - 0.5) * w
                    zr.append(mat_vec(rz, dir_sensor(a, b)))
            per_zone.append(zr)
        rays.append(per_zone)
    return rays


def cast(objects, o, d, ground_d, max_range):
    """First hit (distance, object id) along a world ray, or None."""
    best, bid = max_range, None
    if d[2] > 1e-9:  # going down: ground plane d = ground_d
        t = (ground_d - o[2]) / d[2]
        if 0.0 < t < best:
            best, bid = t, GROUND_ID
    for ob in objects:
        t = ob.ray(o, d, ground_d)
        if t is not None and t < best:
            best, bid = t, ob.id
    return (best, bid) if bid is not None else None


def zone_targets(cfg, hits, rng):
    """Clusters ray hits into targets: [(range_m, coverage, object id)], closest first, <= 2."""
    if not hits:
        return []
    hits.sort()
    clusters, cur = [], [hits[0]]
    for h in hits[1:]:
        if h[0] - cur[-1][0] > cfg.cluster_gap_m:
            clusters.append(cur)
            cur = [h]
        else:
            cur.append(h)
    clusters.append(cur)
    n_rays = cfg.sub * cfg.sub
    out = []
    for c in clusters:
        cov = len(c) / n_rays
        if cov < cfg.min_coverage:
            continue
        r = c[len(c) // 2][0]  # histogram peak ~ median of the cluster
        ids = {}
        for _, oid in c:
            ids[oid] = ids.get(oid, 0) + 1
        oid = max(ids, key=ids.get)
        r += rng.gauss(0.0, max(cfg.noise_min_m, cfg.noise_frac * r))
        out.append((r, cov, oid))
        if len(out) == sil_proto.SIL_TARGETS:
            break
    return out


def make_frame(cfg, rays_s, objects, pose, ground_d, st, rng):
    """One sensor frame: zones for sil_proto.encode_frame and the truth for the log."""
    n_, e_, d_, roll, pitch, yaw = pose
    r_nb = body_to_ned(roll, pitch, yaw)
    o = (n_, e_, d_ - cfg.optics_h_m)
    zones, truth = [], []
    for z, zr in enumerate(rays_s):
        hits = []
        for rb in zr:
            h = cast(objects, o, mat_vec(r_nb, rb), ground_d, cfg.max_range_m)
            if h is not None:
                hits.append(h)
        tg = zone_targets(cfg, hits, rng)
        for ph in st.phantoms:
            if ph[0] == z and ph[2] > 0:
                tg = sorted(tg + [(ph[1], 1.0, "phantom")])[:sil_proto.SIL_TARGETS]
        invalid = st.invalid_frac > 0.0 and rng.random() < st.invalid_frac
        targets = []
        for r, cov, _ in tg:
            status = 4 if invalid else 5  # 4: not a valid status of MAP-03 ({5, 9})
            signal = int(3000.0 * cov / max(r, 0.1) ** 2)
            sigma = int(2.0 + 3.0 * r)
            targets.append((int(round(r * 1000.0)), sigma, signal, status))
        ambient = cfg.ambient * (200 if invalid else 1)
        zones.append((ambient, targets))
        truth.append([[round(r, 3), oid] for r, _, oid in tg] + ([["inv"]] if invalid else []))
    for ph in st.phantoms:
        ph[2] -= 1
    st.phantoms = [ph for ph in st.phantoms if ph[2] > 0]
    return zones, truth


# ---------------------------------------------------------------- runtime

class PoseLink(threading.Thread):
    """ATTITUDE and LOCAL_POSITION_NED from the SITL pose port, kept with arrival times."""

    def __init__(self, url, log=None):
        super().__init__(daemon=True)
        from pymavlink import mavutil
        self.m = mavutil.mavlink_connection(url, source_system=250, source_component=1)
        self.lock = threading.Lock()
        self.hist = []  # (t, n, e, d, roll, pitch, yaw)
        self.att = None
        self.pos = None
        self.ground_d = None
        self.landed = None
        self.stop = False

    def run(self):
        m = self.m
        m.wait_heartbeat(timeout=60)
        for mid, hz in ((30, 50), (32, 50), (245, 5)):
            m.mav.command_long_send(m.target_system, 1, 511, 0, mid, 1e6 / hz, 0, 0, 0, 0, 0)
        while not self.stop:
            msg = m.recv_match(type=["ATTITUDE", "LOCAL_POSITION_NED", "EXTENDED_SYS_STATE"], blocking=True,
                               timeout=0.5)
            if msg is None:
                continue
            t = time.time()
            typ = msg.get_type()
            with self.lock:
                if typ == "ATTITUDE":
                    self.att = (msg.roll, msg.pitch, msg.yaw)
                elif typ == "LOCAL_POSITION_NED":
                    self.pos = (msg.x, msg.y, msg.z)
                    if self.ground_d is None or self.landed == 1:  # MAV_LANDED_STATE_ON_GROUND
                        self.ground_d = msg.z
                else:
                    self.landed = msg.landed_state
                if self.att is not None:
                    # Before the EKF origin is set the vehicle is on the ground at Home.
                    self.hist.append((t,) + (self.pos or (0.0, 0.0, 0.0)) + self.att)
                    if len(self.hist) > 400:
                        del self.hist[:200]

    def pose_at(self, t):
        with self.lock:
            if not self.hist:
                return None, None
            ground_d = 0.0 if self.ground_d is None else self.ground_d
            for s in reversed(self.hist):
                if s[0] <= t:
                    return s[1:], ground_d
            return self.hist[0][1:], ground_d


class Scene:
    def __init__(self, config):
        self.cfg = SensorConfig(config.get("sensor"))
        self.objects = [make_object(o) for o in config.get("objects", [])]
        self.lock = threading.Lock()
        self.st = [SensorState() for _ in self.cfg.azimuths_deg]
        self.drop_prob = config.get("drop_prob", 0.0)
        self.delay_ms = config.get("delay_ms", 0.0)
        self.rays = precompute_rays(self.cfg)
        self.rng = random.Random(self.cfg.seed)

    def clearance(self, n, e, h):
        """Horizontal clearance of a point at height h to the nearest object."""
        best, bid = float("inf"), None
        for ob in self.objects:
            c = ob.clearance(n, e, h)
            if c < best:
                best, bid = c, ob.id
        return best, bid

    def command(self, c):
        with self.lock:
            k = c.get("cmd")
            if k == "add":
                self.objects = [o for o in self.objects if o.id != c["obj"]["id"]] + [make_object(c["obj"])]
            elif k == "remove":
                self.objects = [o for o in self.objects if o.id != c["id"]]
            elif k == "sensor":
                self.st[c["sensor"]].powered = bool(c["powered"])
            elif k == "invalid":
                self.st[c["sensor"]].invalid_frac = float(c["frac"])
            elif k == "phantom":
                for z in c["zones"]:
                    self.st[c["sensor"]].phantoms.append([z, float(c["dist"]), int(c["frames"])])
            elif k == "drop":
                self.drop_prob = float(c["prob"])
            elif k == "delay":
                self.delay_ms = float(c["ms"])
            else:
                return False
        return True


class Runner:
    def __init__(self, scene, pose, sil_addr, log_path, control_port=None):
        self.scene, self.pose = scene, pose
        self.sil = sil_addr
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # The log goes through a writer thread: a write to a slow file system (WSL /mnt) must not stall frames.
        self.log = open(log_path, "w", buffering=1 << 16) if log_path else None
        self.logq = queue.SimpleQueue()
        if self.log:
            threading.Thread(target=self._writer, daemon=True).start()
        self.ctl = None
        if control_port:
            self.ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.ctl.bind(("127.0.0.1", control_port))
            self.ctl.setblocking(False)
        self.quit = False
        self.pending = []  # (send_time, bytes) for latency

    def _writer(self):
        while True:
            line = self.logq.get()
            if line is None:
                self.log.flush()
                return
            self.log.write(line)
            if self.logq.empty():
                self.log.flush()

    def w(self, rec):
        if self.log:
            self.logq.put(json.dumps(rec, separators=(",", ":")) + "\n")

    def poll_control(self):
        while self.ctl:
            try:
                data, _ = self.ctl.recvfrom(65536)
            except BlockingIOError:
                return
            try:
                c = json.loads(data)
            except ValueError:
                continue
            if c.get("cmd") == "quit":
                self.quit = True
                continue
            if c.get("cmd") == "sil":  # forward to tof_sil: control packet
                self.tx.sendto(sil_proto.encode_control(int(c["code"]), int(c.get("arg", 0))), self.sil)
            elif c.get("cmd") == "sensor":
                self.scene.command(c)
                self.tx.sendto(sil_proto.encode_sensor(c["sensor"], c["powered"], int(c.get("spi_errors", 0))),
                               self.sil)
            else:
                self.scene.command(c)
            self.w({"k": "cmd", "t": time.time(), "c": c})

    def run(self):
        sc = self.scene
        cfg = sc.cfg
        period = 1.0 / cfg.freq_hz
        exposure = (cfg.exposure_ms / 1000.0) if cfg.exposure_ms else period
        self.w({"k": "start", "t": time.time(), "sensor": vars(cfg),
                "objects": [o.to_json() for o in sc.objects]})
        t_next = time.time()
        t_pose_log = 0.0
        while not self.quit:
            self.poll_control()
            now = time.time()
            for p in [p for p in self.pending if p[0] <= now]:
                self.tx.sendto(p[1], self.sil)
            self.pending = [p for p in self.pending if p[0] > now]
            if now < t_next:
                time.sleep(min(0.002, t_next - now))
                continue
            t_frame = t_next
            t_next += period
            if t_next < now:  # fell behind: skip ticks, keep the rate
                t_next = now + period
            pose, ground_d = self.pose.pose_at(t_frame - exposure / 2.0)
            if pose is None or ground_d is None:
                continue
            with sc.lock:
                objects = list(sc.objects)
                drop, delay = sc.drop_prob, sc.delay_ms
                for k, st in enumerate(sc.st):
                    st.frame_no += 1
                    if not st.powered:
                        continue
                    zones, truth = make_frame(cfg, sc.rays[k], objects, pose, ground_d, st, sc.rng)
                    if drop > 0.0 and sc.rng.random() < drop:
                        self.w({"k": "drop", "t": t_frame, "s": k, "no": st.frame_no})
                        continue
                    pkt = sil_proto.encode_frame(k, cfg.res, cfg.freq_hz, st.frame_no, zones)
                    if delay > 0.0:
                        self.pending.append((time.time() + delay / 1000.0, pkt))
                    else:
                        self.tx.sendto(pkt, self.sil)
                    self.w({"k": "frame", "t": t_frame, "s": k, "no": st.frame_no, "z": truth})
            if t_frame - t_pose_log >= 0.1:
                t_pose_log = t_frame
                n_, e_, d_, roll, pitch, yaw = pose
                h = ground_d - d_
                clr, oid = sc.clearance(n_, e_, h + cfg.optics_h_m)
                self.w({"k": "pose", "t": t_frame, "n": round(n_, 3), "e": round(e_, 3), "h": round(h, 3),
                        "roll": round(roll, 4), "pitch": round(pitch, 4), "yaw": round(yaw, 4),
                        "clr": None if clr == float("inf") else round(clr, 3), "obj": oid})
        self.w({"k": "stop", "t": time.time()})
        if self.log:
            self.logq.put(None)
            time.sleep(0.5)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True, help="scene JSON: sensor, objects, drop_prob, delay_ms")
    ap.add_argument("--pose", default="tcp:127.0.0.1:5812", help="SITL pose port (pymavlink URL)")
    ap.add_argument("--sil", default="127.0.0.1:14660", help="tof_sil UDP address")
    ap.add_argument("--control", type=int, default=0, help="UDP port for JSON commands")
    ap.add_argument("--log", default=None, help="JSON Lines truth log")
    a = ap.parse_args(argv)
    with open(a.config) as f:
        config = json.load(f)
    host, port = a.sil.split(":")
    pose = PoseLink(a.pose)
    pose.start()
    runner = Runner(Scene(config), pose, (host, int(port)), a.log, a.control)
    try:
        runner.run()
    except KeyboardInterrupt:
        pass
    pose.stop = True
    return 0


if __name__ == "__main__":
    sys.exit(main())
