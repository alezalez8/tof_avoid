"""SITL harness for the E3 scenarios: ArduCopter 4.7.2 SITL (instance 5), the module (tof_sil), the scene
(scene.py) and a test GCS. One scenario = one run directory build/sitl_run/<name>/ with every log.

Ports of instance 5: SERIAL0 5810 test GCS, SERIAL1 5812 scene pose link, SERIAL2 5813 module UART.
The ArduPilot tree is only read: the binary and default parameter files are used as built.
"""

import json
import math
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RUN_ROOT = ROOT / "build" / "sitl_run"
INSTANCE = 5
GCS_PORT = 5760 + 10 * INSTANCE
POSE_PORT = GCS_PORT + 2
MODULE_PORT = GCS_PORT + 3
SIL_UDP = 14660
SCENE_CTL = 14661
HOME = (-35.363261, 149.165230, 584.0, 0.0)  # lat, lon, alt, heading: nose north
FC_SYSID = 1
MODULE_COMPID = 196

COPTER_MODES = {"STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4, "LOITER": 5, "RTL": 6,
                "LAND": 9, "POSHOLD": 16, "BRAKE": 17}
MODE_NAMES = {v: k for k, v in COPTER_MODES.items()}
PHASES = ["UNKNOWN", "GROUND", "ARMED_GROUND", "TAKEOFF", "FLIGHT", "LANDING", "GROUND_RECOVER"]
PRX_BIT = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_PROXIMITY
PREARM_BIT = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
VEHICLE_RADIUS_M = 0.55  # hexa 12": motor circle ~0.8 m + props


def ardupilot_dir():
    env = os.environ.get("TOF_ARDUPILOT")
    if env:
        return Path(env).expanduser()
    sv = shutil.which("sim_vehicle.py")
    if sv:
        return Path(sv).resolve().parents[2]
    return Path.home() / "ardupilot-4.7"


def tof_sil_bin():
    return Path(os.environ.get("TOF_SIL_BIN", ROOT / "build" / "host-debug" / "tools" / "emulator" / "tof_sil" /
                               "tof_sil"))


def ned_to_latlon(n, e):
    lat0, lon0 = HOME[0], HOME[1]
    r = 6378137.0
    return lat0 + math.degrees(n / r), lon0 + math.degrees(e / (r * math.cos(math.radians(lat0))))


def latlon_to_ned(lat, lon):
    r = 6378137.0
    return (math.radians(lat - HOME[0]) * r, math.radians(lon - HOME[1]) * r * math.cos(math.radians(HOME[0])))


def wait_port(port, timeout=30.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.5):
                return True
        except OSError:
            time.sleep(0.2)
    return False


def jsonl(path):
    out = []
    p = Path(path)
    if not p.exists():
        return out
    with open(p) as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    out.append(json.loads(line))
                except ValueError:
                    pass
    return out


def write_param_file(path, params):
    with open(path, "w") as f:
        for k, v in params.items():
            f.write("%s %s\n" % (k, v))


def param_file_crc(paths):
    """CRC-32 of the parameter files in load order (configuration ID for the reports)."""
    import zlib
    crc = 0
    for p in paths:
        crc = zlib.crc32(Path(p).read_bytes(), crc)
    return "%08x" % crc


# ---------------------------------------------------------------- processes

class Sitl:
    def __init__(self, run_dir, params=None, model="hexa", home=HOME):
        self.dir = Path(run_dir)
        self.params = params or {}
        self.model = model
        self.home = home
        self.proc = None
        self.param_files = []

    def start(self):
        ap = ardupilot_dir()
        binary = ap / "build" / "sitl" / "bin" / "arducopter"
        d = ap / "Tools" / "autotest" / "default_params"
        files = [d / "copter.parm", d / "copter-hexa.parm", ROOT / "params" / "sitl_tof.param"]
        if self.params:
            extra = self.dir / "scenario.param"
            write_param_file(extra, self.params)
            files.append(extra)
        self.param_files = files
        cmd = [str(binary), "-S", "--model", self.model, "--speedup", "1", "-I", str(INSTANCE), "-w",
               "--defaults", ",".join(str(f) for f in files),
               "--home", ",".join(str(x) for x in self.home)]
        (self.dir / "sitl_cmd.txt").write_text(" ".join(cmd) + "\n")
        self.out = open(self.dir / "sitl.out", "w")
        self.proc = subprocess.Popen(cmd, cwd=self.dir, stdout=self.out, stderr=subprocess.STDOUT,
                                     start_new_session=True)
        if not wait_port(GCS_PORT, 30):
            raise RuntimeError("SITL did not open port %d" % GCS_PORT)

    def stop(self):
        if self.proc and self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGTERM)
            try:
                self.proc.wait(5)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
        self.proc = None

    def bin_logs(self):
        return sorted((self.dir / "logs").glob("*.BIN"))


class Module:
    """tof_sil with a parameter flash file provisioned once per run (as the bench would)."""

    def __init__(self, run_dir, params=None, blank_flash=False):
        self.dir = Path(run_dir)
        self.params = params or {}
        self.flash = self.dir / "module_flash.bin"
        self.blank = blank_flash
        self.proc = None
        self.boot = 0
        self.logs = []
        self.ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def provision(self):
        if self.blank:
            return
        sets = []
        for k, v in self.params.items():
            sets += ["--set", "%s=%s" % (k, v)]
        r = subprocess.run([str(tof_sil_bin()), "--flash", str(self.flash), "--provision"] + sets,
                           capture_output=True, text=True)
        if r.returncode != 0:
            raise RuntimeError("tof_sil provision failed: " + r.stderr)

    def start(self):
        self.boot += 1
        log = self.dir / ("module_%d.jsonl" % self.boot)
        self.logs.append(log)
        self.err = open(self.dir / ("module_%d.err" % self.boot), "w")
        cmd = [str(tof_sil_bin()), "--fc", "127.0.0.1:%d" % MODULE_PORT, "--udp", str(SIL_UDP), "--log", str(log),
               "--flash", str(self.flash)]
        self.proc = subprocess.Popen(cmd, cwd=self.dir, stdout=subprocess.DEVNULL, stderr=self.err)
        self.t_start = time.time()

    def stop(self, kill=False):
        if self.proc and self.proc.poll() is None:
            if kill:
                self.proc.kill()
            else:
                self.proc.terminate()
            self.proc.wait(5)
        self.proc = None

    def control(self, code, arg=0):
        import sil_proto
        self.ctl.sendto(sil_proto.encode_control(code, arg), ("127.0.0.1", SIL_UDP))

    def records(self, boot=None):
        if boot is None:
            out = []
            for k in range(len(self.logs)):
                out += [dict(r, boot=k + 1) for r in jsonl(self.logs[k])]
            return out
        return jsonl(self.logs[boot - 1])


class SceneProc:
    def __init__(self, run_dir, config):
        self.dir = Path(run_dir)
        self.config = config
        self.proc = None
        self.ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.log = self.dir / "scene.jsonl"

    def start(self):
        cfg = self.dir / "scene.json"
        cfg.write_text(json.dumps(self.config, indent=1))
        cmd = [sys.executable, str(HERE / "scene.py"), "--config", str(cfg), "--pose",
               "tcp:127.0.0.1:%d" % POSE_PORT, "--sil", "127.0.0.1:%d" % SIL_UDP, "--control", str(SCENE_CTL),
               "--log", str(self.log)]
        self.err = open(self.dir / "scene.err", "w")
        self.proc = subprocess.Popen(cmd, cwd=HERE, stdout=self.err, stderr=subprocess.STDOUT)

    def cmd(self, **c):
        self.ctl.sendto(json.dumps(c).encode(), ("127.0.0.1", SCENE_CTL))

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.cmd(cmd="quit")
            try:
                self.proc.wait(3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None

    def records(self):
        return jsonl(self.log)


# ---------------------------------------------------------------- GCS

class Gcs:
    """Test GCS on SERIAL0: background reader, 1 Hz HEARTBEAT and RC override, helpers."""

    KEEP = {"HEARTBEAT", "STATUSTEXT", "COMMAND_ACK", "LOCAL_POSITION_NED", "SYS_STATUS", "EXTENDED_SYS_STATE",
            "ADSB_VEHICLE", "OBSTACLE_DISTANCE", "DISTANCE_SENSOR", "MISSION_CURRENT", "MISSION_ITEM_REACHED",
            "ATTITUDE", "VFR_HUD", "RC_CHANNELS", "MISSION_REQUEST_INT",
            "MISSION_REQUEST", "MISSION_ACK"}

    def __init__(self, run_dir):
        self.dir = Path(run_dir)
        self.m = None
        for _ in range(50):
            try:
                self.m = mavutil.mavlink_connection("tcp:127.0.0.1:%d" % GCS_PORT, source_system=255,
                                                    source_component=190, retries=1)
                break
            except OSError:
                time.sleep(0.2)
        if self.m is None:
            raise RuntimeError("GCS: no connection")
        self.m.setup_logfile(str(self.dir / "gcs.tlog"), mode="wb")
        self.lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.hist = {k: [] for k in self.KEEP}
        self.last = {}
        self.rc = {}  # channel -> pwm override
        self.stop = False
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()
        self.beater = threading.Thread(target=self._beat, daemon=True)
        self.beater.start()

    # --- background
    def _read(self):
        while not self.stop:
            try:
                msg = self.m.recv_msg()
            except (OSError, ValueError):
                time.sleep(0.01)
                continue
            if msg is None:
                time.sleep(0.002)
                continue
            typ = msg.get_type()
            if typ == "BAD_DATA":
                continue
            t = time.time()
            key = (msg.get_srcSystem(), msg.get_srcComponent(), typ)
            with self.lock:
                self.last[key] = (t, msg)
                if typ in self.KEEP:
                    self.hist[typ].append((t, msg))

    def _beat(self):
        while not self.stop:
            with self.send_lock:
                self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                          0)
                if self.rc:
                    ch = [self.rc.get(k + 1, 0) for k in range(18)]
                    self.m.mav.rc_channels_override_send(FC_SYSID, 1, *ch)
            time.sleep(0.2 if self.rc else 1.0)

    def close(self):
        self.stop = True
        time.sleep(0.3)
        try:
            self.m.close()
        except OSError:
            pass

    # --- queries
    def history(self, typ, src_comp=None, since=0.0, pred=None):
        with self.lock:
            h = list(self.hist[typ])
        return [(t, m) for t, m in h if t >= since and (src_comp is None or m.get_srcComponent() == src_comp)
                and (pred is None or pred(m))]

    def latest(self, typ, src_comp=1):
        with self.lock:
            v = self.last.get((FC_SYSID, src_comp, typ))
        return v[1] if v else None

    def fc_heartbeats(self, since=0.0):
        return self.history("HEARTBEAT", 1, since)

    def mode(self):
        hb = self.latest("HEARTBEAT")
        return MODE_NAMES.get(hb.custom_mode, str(hb.custom_mode)) if hb else None

    def armed(self):
        hb = self.latest("HEARTBEAT")
        return bool(hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def pos(self):
        p = self.latest("LOCAL_POSITION_NED")
        return (p.x, p.y, p.z) if p else None

    def texts(self, since=0.0, src_comp=None):
        return [(t, m.get_srcComponent(), m.severity, m.text) for t, m in self.history("STATUSTEXT", src_comp, since)]

    def wait(self, pred, timeout, what="condition", poll=0.1):
        t0 = time.time()
        while time.time() - t0 < timeout:
            v = pred()
            if v:
                return v
            time.sleep(poll)
        raise TimeoutError("timeout %.0f s waiting for %s" % (timeout, what))

    # --- commands
    def command(self, cmd, *p, target_comp=1):
        p = list(p) + [0.0] * (7 - len(p))
        with self.send_lock:
            self.m.mav.command_long_send(FC_SYSID, target_comp, cmd, 0, *p)

    def interval(self, msg_id, hz):
        self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz if hz > 0 else -1)

    def set_param(self, name, value):
        with self.send_lock:
            self.m.mav.param_set_send(FC_SYSID, 1, name.encode(), float(value),
                                      mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def set_mode(self, name, wait=True, timeout=5.0):
        mid = COPTER_MODES[name]
        self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mid)
        if wait:
            self.wait(lambda: self.mode() == name, timeout, "mode " + name)

    def arm(self, timeout=60.0):
        def try_arm():
            if self.armed():
                return True
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            time.sleep(1.0)
            return self.armed()
        self.wait(try_arm, timeout, "armed", poll=0.5)

    def disarm(self, force=False):
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196 if force else 0)

    def takeoff(self, alt, timeout=40.0):
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt)
        self.wait(lambda: self.pos() and -self.pos()[2] > alt - 0.3, timeout, "takeoff to %.1f m" % alt)

    def goto_ned(self, n, e, alt, yaw=None):
        """GUIDED position target, local NED (alt above the EKF origin)."""
        mask = 0b0000101111111000 if yaw is None else 0b0000100111111000
        with self.send_lock:
            self.m.mav.set_position_target_local_ned_send(0, FC_SYSID, 1, mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
                                                          n, e, -alt, 0, 0, 0, 0, 0, 0, yaw or 0.0, 0)

    def rc_override(self, **ch):
        """rc_override(c3=1500): values held and repeated at 5 Hz; 0 releases a channel."""
        for k, v in ch.items():
            self.rc[int(k[1:])] = v

    def upload_mission(self, items, timeout=15.0):
        """items: list of (command, n, e, alt, p1) in local metres from Home; item 0 is Home."""
        wps = [(16, 0.0, 0.0, 0.0, 0.0)] + list(items)
        t_start = time.time()
        with self.send_lock:
            self.m.mav.mission_count_send(FC_SYSID, 1, len(wps), mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        handled = set()
        while time.time() - t_start < timeout:
            for t, a in self.history("MISSION_ACK", 1, t_start):
                if a.type != 0:
                    raise RuntimeError("mission rejected: %d" % a.type)
                return True
            reqs = self.history("MISSION_REQUEST_INT", 1, t_start) + self.history("MISSION_REQUEST", 1, t_start)
            for t, r in reqs:
                if (t, r.seq) in handled:
                    continue
                handled.add((t, r.seq))
                cmd, n, e, alt, p1 = wps[r.seq]
                lat, lon = ned_to_latlon(n, e)
                frame = (mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT if r.seq > 0
                         else mavutil.mavlink.MAV_FRAME_GLOBAL_INT)
                with self.send_lock:
                    self.m.mav.mission_item_int_send(FC_SYSID, 1, r.seq, frame, cmd, 0, 1, p1, 0, 0, 0,
                                                     int(lat * 1e7), int(lon * 1e7), alt,
                                                     mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            time.sleep(0.02)
        raise TimeoutError("mission upload")


# ---------------------------------------------------------------- the whole stand

class Stand:
    """SITL + module + scene + GCS for one scenario."""

    def __init__(self, name, fc_params=None, module_params=None, scene=None, blank_flash=False):
        self.name = name
        self.dir = RUN_ROOT / name
        if self.dir.exists():
            shutil.rmtree(self.dir)
        self.dir.mkdir(parents=True)
        self.sitl = Sitl(self.dir, fc_params)
        base = json.loads((ROOT / "params" / "module_sitl.json").read_text())
        mp = {k: v for k, v in base.items() if not k.startswith("_")}
        mp.update(module_params or {})
        self.module = Module(self.dir, mp, blank_flash)
        self.scene = SceneProc(self.dir, scene or {"objects": []})
        self.gcs = None
        self.t0 = time.time()

    def start(self, module=True, scene=True, wait_ready=True):
        self.module.provision()
        self.sitl.start()
        self.gcs = Gcs(self.dir)  # SERIAL0 waits for this connection
        self.gcs.wait(lambda: self.gcs.latest("HEARTBEAT") is not None, 30, "FC heartbeat")
        for mid, hz in ((32, 10), (1, 5), (245, 5), (246, 20), (30, 10), (330, 10), (132, 10)):
            self.gcs.interval(mid, hz)
        if scene:
            self.scene.start()
        if module:
            self.module.start()
        if wait_ready:
            self.wait_ready()
        return self

    def wait_ready(self, timeout=90.0):
        """GPS/EKF ready and prearm checks pass (needs PRX Good, i.e. the module running)."""
        def ok():
            s = self.gcs.latest("SYS_STATUS")
            return s and (s.onboard_control_sensors_health & PREARM_BIT)
        self.gcs.wait(ok, timeout, "prearm checks")

    def stop(self):
        if self.gcs:
            self.gcs.close()
        self.scene.stop()
        self.module.stop()
        self.sitl.stop()

    # --- helpers for scenarios
    def module_records(self, kind=None, boot=None):
        r = self.module.records(boot)
        return [x for x in r if kind is None or x.get("k") == kind]

    def module_clock(self, boot=1):
        """Function: module time (us) of a boot -> wall-clock seconds, from its start record."""
        st = [r for r in self.module.records(boot) if r.get("k") == "start"][0]
        wall0, t0 = st["wall_us"] / 1e6, st["t"]
        return lambda t_us: wall0 + ((t_us - t0) & 0xFFFFFFFF) / 1e6

    def summary(self):
        return {"run_dir": str(self.dir), "sitl_cmd": (self.dir / "sitl_cmd.txt").read_text().strip(),
                "fc_params_crc": param_file_crc(self.sitl.param_files)}
