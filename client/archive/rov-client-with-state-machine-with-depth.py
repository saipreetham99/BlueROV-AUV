#!/usr/bin/env python3
"""
Combined topside ROV client (single pygame window):
  * Thruster test panel (always runs, even with no video): pick a motion, it drives one
    DOF at a fixed level for a fixed duration through the SAME mix/packet as the driver,
    then neutral. STOP aborts. Same engine as pool_test.py / real_test_gui.py.
  * Manual gamepad control (JOYSTICK button): left stick Y surge, right stick Y strafe,
    RT heave up / right-stick-X-left heave down, XBOX/BACK yaw, D-pad amp +/-100,
    X light, B tag-flash. Live whenever autonomy is off. Same mix/packet path.
  * Live video (shown inside the window when the stream is up; "NO VIDEO" otherwise),
    with AprilTag (optional) and YOLO (optional, --weights) overlays and a RECORD button
    that saves the clean stream to mp4.
Mode (lan/wifi) is chosen at launch with --wifi and applies to both video and thrusters.
Reads .rov_server_creds.
  python rov_client.py
  python rov_client.py --wifi --weights best.pt
  python rov_client.py --weights best.pt --strategy aqua_strategy
"""

import argparse
import configparser
import os
import inspect
import importlib
import json

# cv2 and pygame both ship SDL2; allow the duplicate class registration quietly.
os.environ.setdefault("OBJC_DISABLE_INITIALIZE_FORK_SAFETY", "YES")
import socket
import struct
import sys
import threading
import time
from collections import defaultdict
from datetime import datetime
import cv2
import numpy as np
import pygame

# Autonomy brain: the SAME class the sim (run_sim.py) and the real sub use.
# Which module is loaded is chosen at launch with --strategy (default below).
# main() calls load_strategy() and fills these globals in before App is built,
# so the manual UI still works if the chosen brain isn't importable.
Strategy = None
BoundingBox = None
HAVE_STRATEGY = False
_STRATEGY_ERR = ""
_STRATEGY_NAME = "strategy_full"  # updated by main() to whatever --strategy picked


def load_strategy(module_name):
    """Import Strategy + BoundingBox from the named module (no .py extension).
    Returns (Strategy, BoundingBox, ok, err). On failure ok is False and the
    autonomy button stays disabled -- the manual UI is unaffected."""
    try:
        mod = importlib.import_module(module_name)
        return mod.Strategy, mod.BoundingBox, True, ""
    except Exception as e:  # noqa: BLE001 - missing file etc. -> autonomy disabled
        return None, None, False, str(e)


# ---------- strategy gains (native tune panel) ----------
# Ported verbatim from tune_gui.py so the docked panel writes the SAME
# strategy_gains.json the brain hot-reloads. Kept in the same folder as this
# file (matches where strategy_full.py looks), so it's the one source of truth.
GAINS_FILE = "strategy_gains.json"
GAINS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), GAINS_FILE)
# key, label, min, max, resolution, description
PARAMS = [
    (
        "yaw_kp",
        "Yaw kp (turn)",
        0.0,
        0.01,
        0.00005,
        "How hard it turns to center the target left/right (higher = snappier).",
    ),
    (
        "heave_kp",
        "Heave kp (up/down)",
        0.0,
        0.01,
        0.00005,
        "How hard it dives/climbs to center the target up/down (higher = faster).",
    ),
    (
        "advance_surge",
        "Advance surge",
        0.0,
        1.0,
        0.01,
        "Forward speed while chasing the target (0-1).",
    ),
    (
        "approach_offset_px",
        "Approach offset (px)",
        -250.0,
        250.0,
        5.0,
        "Hold the target this many px off-centre while closing in, to arrive at an "
        "angle. + = target held right of centre. 0 = centred.",
    ),
    (
        "orbit_strafe",
        "Orbit strafe",
        0.0,
        1.0,
        0.01,
        "Sideways speed while circling the target (0-1).",
    ),
    (
        "max_yaw_error_for_strafe",
        "Max yaw err for strafe",
        0.0,
        320.0,
        1.0,
        "Circling strafe ramps in as it gets within this many px of center.",
    ),
    (
        "orbit_enter_frac",
        "Orbit-enter (frac)",
        0.0,
        0.5,
        0.005,
        "Switch to orbiting once the target fills this share of the frame.",
    ),
    (
        "orbit_exit_ratio",
        "Orbit-exit (x enter)",
        0.0,
        1.0,
        0.01,
        "Go back to chasing if the target shrinks below this x the enter size.",
    ),
    (
        "orbit_hold_frac",
        "Orbit hold (frac)",
        0.0,
        0.5,
        0.005,
        "Box size (frac of frame) the orbit holds -- sets the orbit radius. Keep "
        "above orbit-exit or it'll bounce back to chasing.",
    ),
    (
        "orbit_surge_kp",
        "Orbit radius kp",
        0.0,
        40.0,
        0.5,
        "How hard it corrects distance to the hold size (higher = tighter radius). "
        "0 = no radius hold, orbit spirals outward.",
    ),
    (
        "orbit_flip_s",
        "Orbit flip (s)",
        0.0,
        30.0,
        0.5,
        "Seconds circling with no back in view before reversing orbit direction.",
    ),
    (
        "grace_s",
        "Grace (s)",
        0.0,
        3.0,
        0.1,
        "Seconds to keep going after losing sight of the target before searching.",
    ),
    (
        "search_yaw_command",
        "Search spin rate",
        0.0,
        0.4,
        0.005,
        "Speed of the full-circle spin while searching. Lower if it whips past the "
        "target without locking on.",
    ),
    (
        "search_heave_command",
        "Search depth sweep",
        0.0,
        1.0,
        0.01,
        "Up/down bob amplitude while searching (0-1). 0 = spin flat.",
    ),
    (
        "search_heave_period_s",
        "Search sweep period (s)",
        1.0,
        12.0,
        0.5,
        "Seconds for one full up-down depth-sweep cycle while searching.",
    ),
    (
        "dash_s",
        "Dash time (s)",
        0.0,
        4.0,
        0.1,
        "After each orbit flip, whip around at full strafe for this long to swing "
        "behind the target. 0 = no dash.",
    ),
    (
        "dash_strafe",
        "Dash strafe",
        0.0,
        1.0,
        0.01,
        "Strafe speed during that dash (0-1).",
    ),
    (
        "stuck_reset_s",
        "Stuck-reset (s)",
        0.0,
        30.0,
        0.5,
        "Total time circling with no back in view before running the reset "
        "maneuver. Keep well above orbit-flip.",
    ),
    (
        "reset_up_s",
        "Reset: rise time (s)",
        0.0,
        5.0,
        0.1,
        "Reset phase 1 -- seconds spent rising (lifts off the floor).",
    ),
    (
        "reset_heave",
        "Reset: rise speed",
        0.0,
        1.0,
        0.01,
        "Upward speed during the rise phase (0-1).",
    ),
    (
        "reset_fwd_s",
        "Reset: forward time (s)",
        0.0,
        5.0,
        0.1,
        "Reset phase 2 -- seconds driving forward (breaks a vertical stack).",
    ),
    (
        "reset_fwd_surge",
        "Reset: forward speed",
        0.0,
        1.0,
        0.01,
        "Forward speed during the forward phase (0-1).",
    ),
    (
        "reset_turn_s",
        "Reset: turn time (s)",
        0.0,
        5.0,
        0.1,
        "Reset phase 3 -- seconds spinning to bring the target back in view.",
    ),
    (
        "reset_turn_yaw",
        "Reset: turn rate",
        0.0,
        1.0,
        0.01,
        "Turn rate during the spin-around (0-1).",
    ),
    (
        "search_timeout_s",
        "Search timeout (s)",
        0.0,
        20.0,
        0.5,
        "If searching drags on this long AFTER first contact, re-run the reset "
        "maneuver. The initial hunt is unaffected.",
    ),
]
DEFAULTS = {
    "yaw_kp": 0.0003,
    "heave_kp": 0.0003,
    "advance_surge": 0.8,
    "approach_offset_px": 80.0,
    "orbit_strafe": 0.8,
    "max_yaw_error_for_strafe": 80.0,
    "orbit_enter_frac": 0.05,
    "orbit_exit_ratio": 0.75,
    "orbit_hold_frac": 0.06,
    "orbit_surge_kp": 10.0,
    "orbit_flip_s": 6.0,
    "grace_s": 0.5,
    "search_yaw_command": 0.1,
    "search_heave_command": 0.2,
    "search_heave_period_s": 4.0,
    "dash_s": 1.5,
    "dash_strafe": 1.0,
    "stuck_reset_s": 14.0,
    "reset_up_s": 1.0,
    "reset_heave": 0.5,
    "reset_fwd_s": 1.0,
    "reset_fwd_surge": 0.6,
    "reset_turn_s": 1.0,
    "reset_turn_yaw": 0.8,
    "search_timeout_s": 8.0,
}


def load_gains():
    """Read strategy_gains.json over the defaults. Missing/bad file -> defaults."""
    try:
        with open(GAINS_PATH) as f:
            g = json.load(f)
    except (OSError, ValueError):
        g = {}
    out = dict(DEFAULTS)
    for k in out:
        v = g.get(k, out[k])
        if isinstance(v, (int, float)):
            out[k] = float(v)
    if (
        "yaw_kp" not in g
        and "heave_kp" not in g
        and isinstance(g.get("centering_kp"), (int, float))
    ):
        out["yaw_kp"] = out["heave_kp"] = float(g["centering_kp"])
    return out


def fmt_gain(key, v):
    if key in ("yaw_kp", "heave_kp"):
        return f"{v:.5f}"
    if key in ("search_yaw_command", "orbit_enter_frac", "orbit_hold_frac"):
        return f"{v:.3f}"
    if key in (
        "advance_surge",
        "orbit_strafe",
        "orbit_exit_ratio",
        "grace_s",
        "search_heave_command",
        "dash_strafe",
        "reset_heave",
        "reset_fwd_surge",
        "reset_turn_yaw",
    ):
        return f"{v:.2f}"
    return f"{v:.1f}"


NEUTRAL = 1500
LIGHT_OFF = 1100
LIGHT_ON = 1900
SENSOR_FMT = "<dff"  # (epoch_time, depth_m, yaw_deg) - matches rov_server.py
AMP_MIN = 0
AMP_MAX = 400
JOY_DEADZONE = 0.12  # ignore small stick drift around center

# ---------- depth safety envelope ----------
# The sub is kept inside [depth_min, depth_max] (metres, measured from wherever
# you press ZERO DEPTH). This guard overrides autonomy, the thruster test, AND
# the joystick: no control path can drive the sub past the band by hand.
#
# SIGN CONVENTION -- verify this in the water before trusting the guard:
#   press ZERO at the surface, then push the sub DOWN. The on-screen depth (cm)
#   must go MORE POSITIVE. If it goes negative instead, set DEPTH_INCREASES_DOWN
#   to False (that's the ONLY change needed -- everything else keys off it).
DEPTH_INCREASES_DOWN = True  # True: deeper == larger sensor reading
DEPTH_MARGIN = 0.10  # m: stop allowing "further past" this far before a limit
DEPTH_RECOVER = 0.40  # 0..1 heave used to gently climb/dive back once past a limit

# ---------- depth hold (independent mode: PI loop on the depth sensor) ----------
# DEPTH HOLD parks the sub at a typed target depth. It's a real closed loop, so
# unlike the guard above it has gains. P reacts to how far off target it is; I
# trims the steady offset from buoyancy (this sub floats up, so I settles at a
# small DOWN bias). No D term on purpose -- the depth sensor is noisy and water
# drag already damps things; add one only if it oscillates. All heave output is
# scaled by AMP in the packet, so these gains are relative to AMP: tune them at
# the AMP you'll actually demo with. Start here and adjust.
HOLD_KP = 1.5  # heave per metre of error (0.3 m off -> ~0.45 heave before AMP)
HOLD_KI = 0.4  # heave per metre-second of accumulated error (buoyancy trim)
HOLD_I_LIMIT = 0.5  # cap on the heave the integral alone can command (anti-windup)
HOLD_DEADBAND = 0.02  # m: freeze the integrator within +-2 cm so it won't hunt on noise


# ---------- shared thruster core (identical to pool_test.py) ----------
def clamp(x):
    return max(-1.0, min(1.0, x))


def mix(surge, strafe, heave, yaw):
    fl = clamp(surge - strafe - yaw)
    fr = clamp(surge + strafe + yaw)
    rl = clamp(surge + strafe - yaw)
    rr = clamp(surge - strafe + yaw)
    v1 = clamp(heave)
    v2 = clamp(-heave)
    return fl, fr, rl, rr, v1, v2


def to_pwm(x, amp):
    return int(NEUTRAL + x * amp)


def thruster_packet(thr, amp, light=LIGHT_OFF):
    fl, fr, rl, rr, v1, v2 = thr
    return struct.pack(
        "<7H",
        to_pwm(fl, amp),
        to_pwm(fr, amp),
        to_pwm(rl, amp),
        to_pwm(rr, amp),
        to_pwm(v1, amp),
        to_pwm(v2, amp),
        light,
    )


def neutral_packet(light=LIGHT_OFF):
    return struct.pack("<7H", *([NEUTRAL] * 6), light)


def load_config(mode):
    path = os.path.expanduser(".rov_server_creds")
    cfg = configparser.ConfigParser()
    if not os.path.exists(path) or not cfg.read(path):
        sys.exit(f"\u2717 ERROR: Config file not found or empty at '{path}'")
    try:
        rov_ip = cfg[mode]["rov_ip"]
        thruster_port = cfg.getint("DEFAULT", "thruster_port")
        video_port = cfg.getint("DEFAULT", "video_port")
        sensor_port = cfg.getint("DEFAULT", "imu_and_depth_port")
    except (KeyError, configparser.NoSectionError) as e:
        sys.exit(f"\u2717 ERROR: Missing section or key in config: {e}")
    return rov_ip, thruster_port, video_port, sensor_port


class SensorReceiver(threading.Thread):
    """Receives decoded depth/yaw packets from the sub, holds the latest value, and can
    log a decoded CSV during a test."""

    def __init__(self, port):
        super().__init__(daemon=True)
        self.port = port
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = None  # (t, depth_m, yaw_deg)
        self.last_time = 0.0
        self._csv = None

    def run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("", self.port))
        except OSError as e:
            print(f"[sensors] cannot bind port {self.port}: {e}")
            return
        s.settimeout(0.5)
        size = struct.calcsize(SENSOR_FMT)
        print(f"[sensors] listening on *:{self.port}")
        while not self.stop_event.is_set():
            try:
                data, _ = s.recvfrom(256)
            except socket.timeout:
                continue
            if len(data) != size:
                continue
            t, depth, yaw = struct.unpack(SENSOR_FMT, data)
            with self.lock:
                self.latest = (t, depth, yaw)
                self.last_time = time.time()
                if self._csv is not None:
                    self._csv.write(f"{time.time():.4f},{depth:.4f},{yaw:.3f}\n")
                    self._csv.flush()
        s.close()

    def get(self):
        """Latest (depth_m, yaw_deg), or None if stale/absent."""
        with self.lock:
            if self.latest is None or time.time() - self.last_time > 2.0:
                return None
            return self.latest[1], self.latest[2]

    def start_record(self, path):
        with self.lock:
            self._csv = open(path, "w")
            self._csv.write("t_seconds,depth_m,yaw_deg\n")

    def stop_record(self):
        with self.lock:
            if self._csv is not None:
                self._csv.close()
                self._csv = None


# ---------- video receiver (background thread, renders into pygame) ----------
class Recorder:
    def __init__(self):
        self.on = False
        self.writer = None
        self.fname = None

    def toggle(self):
        if self.on:
            self.stop()
        else:
            self.on = True
            self.writer = None

    def write(self, raw):
        if not self.on:
            return
        if self.writer is None:
            h, w = raw.shape[:2]
            self.fname = f"stream_{datetime.now():%Y%m%d_%H%M%S}.mp4"
            self.writer = cv2.VideoWriter(
                self.fname, cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (w, h)
            )
            print(f"[rec] recording -> {self.fname}")
        self.writer.write(raw)

    def stop(self):
        self.on = False
        if self.writer is not None:
            self.writer.release()
            print(f"[rec] saved {self.fname}")
            self.writer = None


import multiprocessing as mp


def _yolo_worker(weights, conf, in_q, out_q, stop_ev):
    """Runs in a SEPARATE process with a clean interpreter.
    Imports torch/ultralytics here only -- never in the pygame+cv2 process,
    which avoids the native-library (SDL/cv2/torch) collision that segfaults
    when they share one process. Receives BGR frames on in_q, returns
    detection boxes on out_q as a list of (x1, y1, x2, y2, conf).
    """
    try:
        from ultralytics import YOLO

        model = YOLO(weights)
    except Exception as e:
        # Report and exit; parent keeps running with video-only.
        try:
            out_q.put(("__error__", str(e)))
        except Exception:
            pass
        return
    while not stop_ev.is_set():
        try:
            frame = in_q.get(timeout=0.5)
        except Exception:
            continue
        if frame is None:
            break
        try:
            res = model(frame, conf=conf, max_det=1, verbose=False)[0]
            boxes = [(*map(int, b.xyxy[0]), float(b.conf[0])) for b in res.boxes]
        except Exception:
            boxes = []
        # Keep only the freshest result: clear any stale box list first.
        try:
            while not out_q.empty():
                out_q.get_nowait()
        except Exception:
            pass
        try:
            out_q.put(boxes)
        except Exception:
            pass


class VideoReceiver(threading.Thread):
    def __init__(self, server_ip, port, weights=None, conf=0.8, yolo_interval=3):
        super().__init__(daemon=True)
        self.server_ip = server_ip
        self.port = port
        self.conf = conf
        self.yolo_interval = max(1, yolo_interval)  # kept for compatibility; unused now
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.last_time = 0.0
        self.tag_ids = []
        self.rec = Recorder()
        self.tag_det = None
        try:
            from pupil_apriltags import Detector

            self.tag_det = Detector(families="tag36h11")
            print("[tags] AprilTag detection on")
        except ImportError:
            print("[tags] pupil_apriltags not installed - tag overlay off")
        # AprilTag detect() is CPU-heavy; run it every Nth frame (not every frame)
        # so it can't throttle the YOLO/box path. Between detections we reuse and
        # redraw the last result, so tag_ids stays populated for the mission
        # counter and the orbit back_visible signal.
        self.tag_interval = 5  # detect ~6 Hz at 30 fps video
        self._last_tag_ids = []
        self._last_tag_hits = []  # cached (corners, center, id) for the overlay
        # YOLO runs in a separate process (see _yolo_worker). Torch is never
        # imported in this process, so it can't collide with pygame/cv2/SDL.
        self.yolo_on = bool(weights)
        self.yolo_proc = None
        self.yolo_in = None
        self.yolo_out = None
        self.yolo_stop = None
        self.last_boxes = []
        self._auto_boxes = []  # thread-safe copy of last_boxes for the autonomy loop
        self._auto_dims = (0, 0)  # (w, h) of the last decoded frame
        # --- perception latency instrumentation ---
        # _box_ready_t: wall time the current box was popped from the worker, i.e.
        #   the earliest it was available to the rest of the pipeline. A 30 Hz
        #   consumer (UI / autonomy loop) reads it some ms later; THAT gap is the
        #   coupling + timer delay that #2 (drain thread) and #3 (event-driven
        #   control) would remove. It excludes inference, which #2/#3 can't touch.
        # _det_period: smoothed seconds between successive worker results -> det Hz.
        self._box_ready_t = 0.0
        self._det_period = 0.0
        self._last_pop_t = 0.0
        self.box_event = threading.Event()  # set when a fresh box arrives (drives #3)
        if weights:
            ctx = mp.get_context("spawn")  # fresh interpreter, NOT fork
            self.yolo_in = ctx.Queue(maxsize=1)
            self.yolo_out = ctx.Queue(maxsize=1)
            self.yolo_stop = ctx.Event()
            self.yolo_proc = ctx.Process(
                target=_yolo_worker,
                args=(weights, self.conf, self.yolo_in, self.yolo_out, self.yolo_stop),
                daemon=True,
            )
            self.yolo_proc.start()
            print(f"[yolo] worker process started ({weights})")

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("", self.port))
        except OSError as e:
            print(f"[video] cannot bind port {self.port}: {e}")
            return
        sock.settimeout(1.0)
        print(f"[video] listening on *:{self.port} from {self.server_ip}")
        buffers = defaultdict(lambda: {"total": None, "parts": {}, "ts": time.time()})
        rf = 0
        while not self.stop_event.is_set():
            try:
                packet, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            if self.server_ip != "any" and addr[0] != self.server_ip:
                continue
            fid, total, idx = struct.unpack("!HHH", packet[:6])
            buf = buffers[fid]
            buf["ts"] = time.time()
            if buf["total"] is None:
                buf["total"] = total
            buf["parts"][idx] = packet[6:]
            if len(buf["parts"]) == buf["total"]:
                jpg = b"".join(buf["parts"][i] for i in range(buf["total"]))
                del buffers[fid]
                frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue
                # Camera is mounted rotated; make every frame upright BEFORE
                # anything downstream (AprilTags, YOLO, recording, UI, autonomy)
                # sees it, so all orientations stay consistent.
                # frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                raw = frame.copy()
                # Refresh AprilTag detection only every tag_interval-th frame and
                # reuse the cached result in between. This keeps the expensive
                # detect() call off the per-frame path so it can't slow the box
                # down. ids stays populated for the mission counter + back_visible.
                if self.tag_det is not None and rf % self.tag_interval == 0:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    self._last_tag_ids = []
                    self._last_tag_hits = []
                    for tag in self.tag_det.detect(gray):
                        pts = tag.corners.astype(int)
                        ctr = tuple(map(int, tag.center))
                        self._last_tag_hits.append((pts, ctr, int(tag.tag_id)))
                        self._last_tag_ids.append(int(tag.tag_id))
                ids = self._last_tag_ids
                # redraw the cached tags every frame (cheap) so the overlay stays
                # smooth even on the frames we skip detection
                for pts, (cx, cy), tid in self._last_tag_hits:
                    for i in range(4):
                        cv2.line(
                            frame,
                            tuple(pts[i]),
                            tuple(pts[(i + 1) % 4]),
                            (0, 255, 0),
                            2,
                        )
                    cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                    cv2.putText(
                        frame,
                        f"id {tid}",
                        (cx + 8, cy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )
                # Hand the freshest frame to the YOLO worker EVERY frame (no
                # interval gate). The maxsize=1 queue + drop-stale below means a
                # busy worker just skips frames instead of backing up, so the box
                # updates as fast as the worker can produce it.
                if self.yolo_on:
                    try:
                        if self.yolo_in.full():
                            try:
                                self.yolo_in.get_nowait()
                            except Exception:
                                pass
                        self.yolo_in.put_nowait(frame.copy())
                    except Exception:
                        pass
                # Pull any boxes the worker has ready (non-blocking) and cache
                # them so they persist on-screen between inferences.
                got_new_box = False
                pop_t = 0.0
                if self.yolo_on:
                    try:
                        got = self.yolo_out.get_nowait()
                        if isinstance(got, tuple) and got and got[0] == "__error__":
                            print(f"[yolo] worker failed: {got[1]} - detection off")
                            self.yolo_on = False
                        else:
                            self.last_boxes = got
                            got_new_box = True  # a fresh result arrived this frame
                            pop_t = time.time()
                    except Exception:
                        pass
                    for x1, y1, x2, y2, cf in self.last_boxes:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 180, 0), 2)
                        cv2.putText(
                            frame,
                            f"{cf:.2f}",
                            (x1, max(y1 - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (255, 180, 0),
                            2,
                        )
                rf += 1
                self.rec.write(raw)
                with self.lock:
                    self.latest = frame
                    self.last_time = time.time()
                    self.tag_ids = ids
                    self._auto_boxes = list(self.last_boxes)
                    self._auto_dims = (frame.shape[1], frame.shape[0])
                    # advance the box-ready stamp + det-rate ONLY when a fresh box
                    # actually arrived, so "box lag" measures the age of the real
                    # detection, not the time since the last redraw.
                    if got_new_box:
                        if self._last_pop_t:
                            p = pop_t - self._last_pop_t
                            self._det_period = (
                                p
                                if self._det_period == 0.0
                                else 0.9 * self._det_period + 0.1 * p
                            )
                        self._last_pop_t = pop_t
                        self._box_ready_t = pop_t
                        self.box_event.set()  # wake the event-driven control loop
            now = time.time()
            for k in [k for k, v in buffers.items() if now - v["ts"] > 0.5]:
                del buffers[k]
        sock.close()
        self.rec.stop()
        self._stop_yolo()

    def _stop_yolo(self):
        if self.yolo_proc is None:
            return
        try:
            self.yolo_stop.set()
            self.yolo_in.put_nowait(None)  # unblock the worker's get()
        except Exception:
            pass
        self.yolo_proc.join(timeout=2.0)
        if self.yolo_proc.is_alive():
            self.yolo_proc.terminate()
        self.yolo_proc = None

    def get_frame(self):
        with self.lock:
            if self.latest is None or time.time() - self.last_time > 1.5:
                return None
            return self.latest

    def toggle_record(self):
        self.rec.toggle()

    def get_tag_ids(self):
        with self.lock:
            return list(self.tag_ids)

    def get_detection(self):
        """Freshest YOLO box (x1, y1, x2, y2) or None, plus the frame size (w, h).
        Returns None if there's no detection or the video has gone stale (>1.5 s)."""
        with self.lock:
            fresh = self.latest is not None and (time.time() - self.last_time) <= 1.5
            boxes = list(self._auto_boxes)
            dims = self._auto_dims
        if not fresh or not boxes:
            return None, dims
        return boxes[0], dims  # worker runs max_det=1, so one box is all there is

    def get_box_lag(self):
        """(age_ms, det_hz) for the freshest box.
        age_ms = ms since the box left the worker (measured now, at the caller's
        read) -- the post-inference pipeline delay #2/#3 would shave.
        det_hz = smoothed rate at which new boxes arrive (should track video fps).
        Returns (0.0, 0.0) until the first box has been seen."""
        with self.lock:
            t = self._box_ready_t
            period = self._det_period
        age_ms = (time.time() - t) * 1000.0 if t else 0.0
        hz = (1.0 / period) if period > 0 else 0.0
        return age_ms, hz


# ---------- pygame UI ----------
class Button:
    def __init__(self, rect, label, action, color=(70, 90, 120), enabled=None):
        self.rect = pygame.Rect(rect)
        self.label = label
        self.action = action
        self.color = color
        self.enabled = enabled or (lambda: True)

    def text(self):
        return self.label() if callable(self.label) else self.label

    def draw(self, surf, font):
        on = self.enabled()
        pygame.draw.rect(
            surf, self.color if on else (55, 55, 60), self.rect, border_radius=8
        )
        pygame.draw.rect(surf, (200, 200, 200), self.rect, width=2, border_radius=8)
        t = font.render(self.text(), True, (255, 255, 255) if on else (110, 110, 110))
        surf.blit(t, t.get_rect(center=self.rect.center))

    def click(self, pos):
        if self.enabled() and self.rect.collidepoint(pos):
            self.action()
            return True
        return False


class App:
    def __init__(self, mode, weights, conf, yolo_interval):
        self.mode = mode
        self.rov_ip, self.thr_port, self.video_port, self.sensor_port = load_config(
            mode
        )
        self.thr_addr = (self.rov_ip, self.thr_port)
        pygame.init()
        self.W, self.H = 1000, 720
        # --- tune panel: docks to the right; window widens when open ---
        self.PANEL_W = 380
        self.PANEL_X = self.W  # panel starts just past the main area
        self.ROW_H = 44  # per-slider row height inside the scroll list
        self.SCROLL_TOP = 96  # scroll list starts below the header + buttons
        self.SCROLL_BOT = self.H - 84  # ... and ends above the footer
        self.panel_open = False
        self.panel_scroll = 0
        self.slider_drag = None  # key of the slider currently being dragged
        self.gains = load_gains()  # current values, edited by the sliders
        self._gains_dirty = False  # unsaved edits pending a debounced write
        self._gains_last_edit = 0.0
        self.panel_status = f"editing {GAINS_FILE}"
        self.screen = pygame.display.set_mode((self.W, self.H))
        pygame.display.set_caption("ROV Client")
        self.f_title = pygame.font.SysFont("Helvetica", 24, bold=True)
        self.f_btn = pygame.font.SysFont("Helvetica", 18, bold=True)
        self.f_small = pygame.font.SysFont("Helvetica", 15)
        self.f_status = pygame.font.SysFont("Helvetica", 20, bold=True)
        # thruster-test settings/state
        self.duration = 3
        self.amp = 100
        # AMP is a click-to-type text box now (clamped AMP_MIN..AMP_MAX on commit).
        self.amp_rect = pygame.Rect(200, 90, 130, 36)
        self.amp_text = str(self.amp)
        self.amp_active = False
        # "Tags to finish": unique-AprilTag target, click-to-type box.
        self.tag_target = 2
        self.target_rect = pygame.Rect(372, 552, 46, 36)
        self.target_text = str(self.tag_target)
        self.target_active = False
        # Depth safety band (metres), tared by ZERO DEPTH. depth_zero is the RAW
        # sensor reading that counts as "0"; effective_depth() subtracts it.
        # Defaults are a shallow band so the very first joystick test is easy:
        # push down -> stops at 1.5 m, push up -> stops at the surface. SET THESE
        # for your pool, and press ZERO at the surface before you drive.
        self.depth_min = 0.0
        self.depth_max = 1.5
        self.depth_zero = 0.0
        self.depth_min_text = f"{self.depth_min:.2f}"
        self.depth_max_text = f"{self.depth_max:.2f}"
        self.depth_min_active = False
        self.depth_max_active = False
        self.depth_min_rect = pygame.Rect(408, 680, 58, 32)
        self.depth_max_rect = pygame.Rect(512, 680, 58, 32)
        # Depth HOLD: independent mode that parks the sub at a typed target depth
        # (metres from the ZERO). While it's on, every other mode is greyed out and
        # the min/max guard does not apply -- STOP is the backstop. _hold_i is the
        # PI integrator; _hold_last_t gives the loop real dt.
        self.depth_hold = False
        self.hold_target = 0.5
        self.hold_target_text = f"{self.hold_target:.2f}"
        self.hold_target_active = False
        self.hold_target_rect = pygame.Rect(612, 680, 58, 32)
        self._hold_i = 0.0
        self._hold_last_t = 0.0
        self.capture = False
        self.rate = 50.0
        # light state
        self.light_on = False  # manual LIGHT toggle
        self.tag_flash = False  # flash lights while an AprilTag is in view
        self.countdown = 3
        self.running = False
        self.abort = False
        # autonomy (chase & orbit) state
        self.autonomous = False
        self.strategy = None
        self.auto_state = "-"
        self.auto_cmd = (0.0, 0.0, 0.0, 0.0)
        self._auto_thread = None
        # unique-AprilTag termination memory (latched until RESET TAGS):
        # while autonomous, each distinct tag ID seen is remembered; reaching
        # tag_target flashes the lights and hands off to the pad.
        self.collected_tags = set()
        self.mission_complete = False
        self.celebrate_until = 0.0  # lights flash while now < this
        # manual joystick (gamepad) control
        self.joystick_on = False
        self.joy = None
        self.trig_rest = {4: -1.0, 5: -1.0}  # recalibrated when the mode is enabled
        self.joy_cmd = (0.0, 0.0, 0.0, 0.0)
        self._init_joystick()
        self._box_lag_ms = 0.0  # smoothed display box-lag (30 Hz UI read)
        self._ctrl_lag_ms = 0.0  # smoothed control-path box-lag (event-driven loop)
        self.status = "Ready"
        self.log = []
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # video
        self.video = VideoReceiver(
            self.rov_ip, self.video_port, weights, conf, yolo_interval
        )
        self.video.start()
        # sensors (depth + yaw)
        self.sensors = SensorReceiver(self.sensor_port)
        self.sensors.start()
        # video panel geometry
        self.vid_rect = pygame.Rect(370, 20, 610, 458)
        # manual controls are locked out while autonomy OR the gamepad is driving
        idle = lambda: (
            not self.running
            and not self.autonomous
            and not self.joystick_on
            and not self.depth_hold
        )
        self.buttons = []
        self.buttons.append(
            Button((20, 90, 36, 36), "-", lambda: self.set_dur(-1), (80, 80, 90), idle)
        )
        self.buttons.append(
            Button((120, 90, 36, 36), "+", lambda: self.set_dur(+1), (80, 80, 90), idle)
        )
        # (AMP steppers removed -- AMP is now the text box at self.amp_rect)
        self.buttons.append(
            Button(
                (20, 135, 316, 32),
                lambda: f"Sensor capture: {'ON' if self.capture else 'OFF'}",
                self.toggle_capture,
                (90, 110, 90),
                idle,
            )
        )
        grid = [
            ("Forward", "surge", 1),
            ("Backward", "surge", -1),
            ("Strafe Left", "strafe", -1),
            ("Strafe Right", "strafe", 1),
            ("Up", "heave", 1),
            ("Down", "heave", -1),
            ("Yaw Left", "yaw", -1),
            ("Yaw Right", "yaw", 1),
        ]
        bx, by, bw, bh, gx, gy = 20, 185, 155, 50, 6, 12
        for i, (label, motion, sign) in enumerate(grid):
            col, row = i % 2, i // 2
            r = (bx + col * (bw + gx), by + row * (bh + gy), bw, bh)
            self.buttons.append(
                Button(
                    r,
                    label,
                    lambda m=motion, s=sign: self.start(m, s),
                    (70, 90, 120),
                    idle,
                )
            )
        stop_y = by + 4 * (bh + gy) + 4
        self.buttons.append(
            Button(
                (20, stop_y, bw * 2 + gx, 52),
                "STOP",
                self.stop,
                (170, 60, 60),
                lambda: True,
            )
        )
        self.status_y = stop_y + 66
        # manual gamepad: L-stick Y surge, R-stick Y strafe, RT heave up /
        # R-stick X-left heave down, XBOX/BACK yaw, D-pad amp +/-100, X light,
        # B tag-flash.
        self.buttons.append(
            Button(
                (20, 645, 316, 46),
                lambda: (
                    "JOYSTICK: ON"
                    if self.joystick_on
                    else ("JOYSTICK: OFF" if self.joy else "JOYSTICK: NO PAD")
                ),
                self.toggle_joystick,
                (60, 120, 140),
                lambda: (
                    self.joystick_on
                    or (
                        self.joy is not None
                        and not self.autonomous
                        and not self.running
                        and not self.depth_hold
                    )
                ),
            )
        )
        # video record button (below the video panel)
        self.buttons.append(
            Button(
                (370, 486, 180, 44),
                lambda: "STOP REC" if self.video.rec.on else "RECORD",
                self.video.toggle_record,
                (180, 50, 50) if False else (70, 110, 90),
                lambda: True,
            )
        )
        # light + apriltag-flash toggles (green while active) - placed on a row
        # below the tag/sensor readouts so they don't cover the ID text
        self.buttons.append(
            Button(
                (565, 560, 150, 44),
                lambda: "LIGHT: ON" if self.light_on else "LIGHT: OFF",
                self.toggle_light,
                (150, 130, 40),
                lambda: not self.autonomous,  # greyed during a hunt
            )
        )
        self.buttons.append(
            Button(
                (725, 560, 170, 44),
                lambda: "TAG FLASH: ON" if self.tag_flash else "TAG FLASH: OFF",
                self.toggle_tag_flash,
                (120, 90, 140),
                lambda: not self.autonomous,  # greyed during a hunt
            )
        )
        # RESET TAGS: clear collected unique tags + un-latch completion.
        self.buttons.append(
            Button(
                (432, 552, 126, 38),
                "RESET TAGS",
                self.reset_tags,
                (120, 90, 90),
                lambda: True,
            )
        )
        # ZERO DEPTH: snapshot the current depth as the new 0 (press at surface).
        # Greyed during a hold (re-taring mid-hold would move the target).
        self.buttons.append(
            Button(
                (680, 678, 120, 36),
                "ZERO DEPTH",
                self.zero_depth,
                (90, 110, 130),
                lambda: not self.depth_hold,
            )
        )
        # DEPTH HOLD: independent PI depth-hold mode. Enabled only when nothing
        # else is driving and depth data is live; always toggle-OFF-able.
        self.buttons.append(
            Button(
                (806, 678, 150, 36),
                lambda: "HOLD: ON" if self.depth_hold else "DEPTH HOLD",
                self.toggle_depth_hold,
                (70, 130, 120),
                lambda: (
                    self.depth_hold
                    or (
                        not self.running
                        and not self.autonomous
                        and not self.joystick_on
                        and self.effective_depth() is not None
                    )
                ),
            )
        )
        # AUTONOMOUS toggle: YOLO box -> strategy brain -> thrusters.
        # Enabled only with the brain imported + YOLO running; always toggle-OFF-able.
        self.buttons.append(
            Button(
                (370, 614, 250, 46),
                lambda: "AUTONOMOUS: ON" if self.autonomous else "AUTONOMOUS: OFF",
                self.toggle_autonomous,
                (60, 140, 90),
                lambda: (
                    self.autonomous
                    or (
                        (not self.running)
                        and not self.joystick_on
                        and not self.depth_hold
                        and HAVE_STRATEGY
                        and self.video.yolo_on
                    )
                ),
            )
        )
        # TUNE toggle: opens/closes the docked slider panel (widens the window).
        # Same row as RECORD (which ends at x=550), so this sits clear at x=800.
        self.buttons.append(
            Button(
                (800, 486, 175, 44),
                lambda: "TUNE \u25c2 CLOSE" if self.panel_open else "TUNE \u25b8",
                self.toggle_panel,
                (90, 90, 130),
                lambda: True,
            )
        )
        # Panel Reload / Reset live at x >= PANEL_X, so they're only on-screen (and
        # only clickable) while the panel is open and the window is widened.
        self.buttons.append(
            Button(
                (self.PANEL_X + 14, 54, 168, 32),
                "Reload file",
                self._reload_gains,
                (70, 100, 90),
                lambda: self.panel_open,
            )
        )
        self.buttons.append(
            Button(
                (self.PANEL_X + 198, 54, 168, 32),
                "Reset defaults",
                self._reset_gains,
                (110, 90, 70),
                lambda: self.panel_open,
            )
        )
        if not HAVE_STRATEGY:
            print(
                f"[autonomy] {_STRATEGY_NAME} not importable ({_STRATEGY_ERR}); "
                "autonomous mode disabled"
            )

    # ---- tune panel: sliders write strategy_gains.json (brain hot-reloads it) ----
    def toggle_panel(self):
        """Show/hide the docked panel by widening/narrowing the window."""
        self.panel_open = not self.panel_open
        w = self.W + (self.PANEL_W if self.panel_open else 0)
        self.screen = pygame.display.set_mode((w, self.H))

    def _panel_rows(self):
        """Yield (key, label, lo, hi, res, desc, track_x, track_w, content_y) for
        each slider. content_y is the row top before scroll is applied."""
        x = self.PANEL_X + 14
        w = self.PANEL_W - 28
        for i, (key, label, lo, hi, res, desc) in enumerate(PARAMS):
            yield key, label, lo, hi, res, desc, x, w, i * self.ROW_H

    def _row_screen_y(self, content_y):
        """Screen y of a row's TOP given the current scroll offset."""
        return self.SCROLL_TOP - self.panel_scroll + content_y

    def _scroll_max(self):
        content_h = len(PARAMS) * self.ROW_H
        return max(0, content_h - (self.SCROLL_BOT - self.SCROLL_TOP))

    def _set_gain_from_x(self, key, lo, hi, res, mx, track_x, track_w):
        """Map a mouse x on the track to a snapped, clamped value; mark dirty."""
        frac = (mx - track_x) / max(track_w, 1)
        frac = max(0.0, min(1.0, frac))
        v = lo + frac * (hi - lo)
        v = round(v / res) * res
        v = max(lo, min(hi, v))
        self.gains[key] = v
        self._gains_dirty = True
        self._gains_last_edit = time.time()

    def _panel_slider_hit(self, pos):
        """If pos lands on a slider track, start dragging it. Returns True if so."""
        if not self.panel_open:
            return False
        mx, my = pos
        for key, _lbl, lo, hi, res, _d, tx, tw, cy in self._panel_rows():
            ty = self._row_screen_y(cy) + 26  # track centre line
            if ty < self.SCROLL_TOP or ty > self.SCROLL_BOT:
                continue  # scrolled out of the visible list
            if tx - 8 <= mx <= tx + tw + 8 and ty - 12 <= my <= ty + 12:
                self.slider_drag = key
                self._set_gain_from_x(key, lo, hi, res, mx, tx, tw)
                return True
        return False

    def _drag_slider(self, pos):
        """Continue an in-progress drag from a MOUSEMOTION event."""
        key = self.slider_drag
        if key is None:
            return
        for k, _lbl, lo, hi, res, _d, tx, tw, _cy in self._panel_rows():
            if k == key:
                self._set_gain_from_x(key, lo, hi, res, pos[0], tx, tw)
                return

    def _save_gains(self):
        """Write current values to the gains file (the brain reloads on change)."""
        data = {k: round(float(v), 6) for k, v in self.gains.items()}
        try:
            with open(GAINS_PATH, "w") as f:
                json.dump(data, f, indent=2)
            self._gains_dirty = False
            self.panel_status = (
                f"saved  yaw_kp={data['yaw_kp']:.5f}  heave_kp={data['heave_kp']:.5f}"
            )
        except OSError as e:
            self.panel_status = f"write failed: {e}"

    def _reload_gains(self):
        self.gains = load_gains()
        self._gains_dirty = False
        self.panel_status = "reloaded from file"

    def _reset_gains(self):
        self.gains = dict(DEFAULTS)
        self._gains_dirty = True
        self._gains_last_edit = time.time()
        self.panel_status = "reset to defaults"

    # settings
    def toggle_capture(self):
        self.capture = not self.capture

    def toggle_light(self):
        self.light_on = not self.light_on

    def toggle_tag_flash(self):
        self.tag_flash = not self.tag_flash

    # ---- unique-AprilTag target box + collection memory ----
    def target_editable(self):
        """Editable except during a thruster test. Tags-to-finish is read live,
        so it can be changed during a hunt."""
        return not self.running

    def commit_target(self):
        """Parse the typed target, clamp to 1..99, and deactivate the box.
        Empty or non-numeric input falls back to the current target."""
        try:
            v = int(self.target_text)
        except (ValueError, TypeError):
            v = self.tag_target
        v = max(1, min(99, v))
        self.tag_target = v
        self.target_text = str(v)
        self.target_active = False

    def reset_tags(self):
        """Clear the collected unique tags and un-latch mission complete."""
        self.collected_tags.clear()
        self.mission_complete = False
        self.celebrate_until = 0.0
        self.set_status("tag collection reset (0 collected)")
        self.add_log("tags reset")

    def _complete_mission(self):
        """Target reached: flash the lights, reset the manual light state, stop
        autonomy, and hand off to the pad."""
        self.mission_complete = True
        self.celebrate_until = time.time() + 3.0  # ~3 s celebration flash
        # reset the manual light toggles so control hands off to the pad clean:
        # after the celebration blink the light is off and nothing is strobing.
        self.light_on = False
        self.tag_flash = False
        got = sorted(self.collected_tags)
        self._stop_autonomy()  # end chase & orbit, sub -> neutral
        self.add_log(f"COMPLETE {len(got)} unique tags {got}")
        if self.joy is not None:
            self._start_joystick()  # auto hand-off to the gamepad
            self.set_status(f">>> COMPLETE ({len(got)} tags) - JOYSTICK on")
        else:
            self.set_status(f">>> COMPLETE ({len(got)} tags) - no pad connected")

    def current_light(self):
        """PWM to put in the packet's light channel right now.
        Priority: the win celebration flash overrides everything; then, while
        autonomous, the light stays off; otherwise TAG FLASH blinks when a tag is
        seen, else the manual LIGHT toggle wins.
        """
        if time.time() < self.celebrate_until:
            # ~4 Hz celebration blink
            return LIGHT_ON if int(time.time() * 4) % 2 == 0 else LIGHT_OFF
        # While autonomous the light stays off: LIGHT/TAG FLASH are greyed out and
        # don't affect the win, so the win celebration above is the only light
        # event during a hunt.
        if self.autonomous:
            return LIGHT_OFF
        if self.tag_flash and self.video.get_tag_ids():
            # ~4 Hz blink: on/off every 0.25 s
            return LIGHT_ON if int(time.time() * 4) % 2 == 0 else LIGHT_OFF
        return LIGHT_ON if self.light_on else LIGHT_OFF

    # ---- autonomous chase & orbit (topside): YOLO box -> Strategy -> thrusters ----
    def toggle_autonomous(self):
        if self.autonomous:
            self._stop_autonomy()
        elif (
            (not self.running)
            and not self.joystick_on
            and HAVE_STRATEGY
            and self.video.yolo_on
        ):
            self._start_autonomy()

    def _start_autonomy(self):
        # Build the brain in the SAME 640x480 frame it was tuned in; the worker
        # scales each YOLO box into that frame, so strategy_gains.json transfers
        # from the sim unchanged.
        # Fresh collection each run: restarting autonomy clears any latched tags.
        self.collected_tags.clear()
        self.mission_complete = False
        self.celebrate_until = 0.0
        self.strategy = Strategy(camera_width=640, camera_height=480)
        # Some brains take (box, dt, back_visible); older ones take (box, dt).
        # Check the signature once so the worker calls each correctly.
        self._brain_wants_back = (
            len(inspect.signature(self.strategy.update).parameters) >= 3
        )
        self.abort = False
        self.autonomous = True
        self.auto_state = self.strategy.state
        self._auto_thread = threading.Thread(target=self._autonomy_worker, daemon=True)
        self._auto_thread.start()
        self.set_status(">>> AUTONOMOUS - chase & orbit")
        self.add_log(f"autonomous ON  (amp={self.amp})")

    def _stop_autonomy(self):
        self.autonomous = False
        self._ctrl_lag_ms = 0.0  # clear the control-path readout
        for _ in range(5):
            try:
                self.sock.sendto(neutral_packet(self.current_light()), self.thr_addr)
            except Exception:
                pass
        self.set_status("AUTONOMOUS off - neutral sent")
        self.add_log("autonomous OFF")

    def _autonomy_worker(self):
        # Event-driven (#3): block until a fresh YOLO box arrives, then run exactly
        # one control step on it. This removes the old fixed-30 Hz timer that kept
        # re-sampling stale boxes (the ~31 ms "box lag"). The wait has a timeout so
        # that if video stalls we still tick -- the sub keeps searching, stays
        # under command (well inside the server's 0.5 s neutral watchdog), and we
        # re-check the abort/autonomous flags. dt is real elapsed time between
        # steps, which the strategy's timers want anyway.
        WAIT_TIMEOUT = 0.1  # s; also the keep-alive floor when no boxes are coming
        self.video.box_event.clear()  # don't act on a stale pre-start signal
        last = time.time()
        while self.autonomous and not self.abort:
            self.video.box_event.wait(timeout=WAIT_TIMEOUT)
            self.video.box_event.clear()
            now = time.time()
            dt = now - last
            last = now
            raw, (fw, fh) = self.video.get_detection()
            # box age measured right at the control read -> this is the number #3
            # drives down (from ~31 ms toward single digits). Shown as "ctrl lag".
            lag_ms, _ = self.video.get_box_lag()
            if raw is not None and fw and fh:
                x1, y1, x2, y2 = raw[:4]
                sx, sy = 640.0 / fw, 480.0 / fh  # -> the strategy's tuning frame
                box = BoundingBox(
                    x=x1 * sx,
                    y=y1 * sy,
                    width=(x2 - x1) * sx,
                    height=(y2 - y1) * sy,
                )
            else:
                box = BoundingBox()  # nothing seen -> the brain searches
            # Tags live on the back, so "a tag is in view" == "I'm looking at the
            # back". Feed that in so ORBITING knows when to stop and SCAN.
            back_visible = bool(self.video.get_tag_ids())
            if self._brain_wants_back:
                surge, strafe, heave, yaw, flash = self.strategy.update(
                    box, dt, back_visible
                )
            else:
                surge, strafe, heave, yaw, flash = self.strategy.update(box, dt)
            heave = self.guard_heave(heave)  # depth band overrides the strategy
            thr = mix(surge, strafe, heave, yaw)
            light = LIGHT_ON if flash else self.current_light()  # blink on a win
            try:
                self.sock.sendto(thruster_packet(thr, self.amp, light), self.thr_addr)
            except Exception:
                pass
            with self.lock:
                self.auto_state = self.strategy.state
                self.auto_cmd = (surge, strafe, heave, yaw)
                self._ctrl_lag_ms = (
                    lag_ms
                    if self._ctrl_lag_ms == 0.0
                    else 0.85 * self._ctrl_lag_ms + 0.15 * lag_ms
                )
        # left the loop -> make sure the sub is neutral
        for _ in range(3):
            try:
                self.sock.sendto(neutral_packet(self.current_light()), self.thr_addr)
            except Exception:
                pass

    # ---- manual gamepad control ----
    def _init_joystick(self):
        """Grab joystick 0 if present. Safe to re-call to detect a pad plugged in later."""
        try:
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joy = pygame.joystick.Joystick(0)
                self.joy.init()
                print(
                    f"[pad] {self.joy.get_name()}  "
                    f"axes={self.joy.get_numaxes()} buttons={self.joy.get_numbuttons()}"
                )
            else:
                self.joy = None
                print("[pad] no controller detected")
        except pygame.error as e:
            self.joy = None
            print(f"[pad] init failed: {e}")

    def toggle_joystick(self):
        if self.joystick_on:
            self._stop_joystick()
        elif self.joy is not None and not self.autonomous and not self.running:
            self._start_joystick()

    def _start_joystick(self):
        if self.joy is None:
            self._init_joystick()
        if self.joy is None:
            self.set_status("JOYSTICK: no controller found")
            return
        self.abort = False
        # Snapshot RT's resting value so heave-up is 0 when released, whether
        # this pad rests its triggers at -1 or 0.
        pygame.event.pump()
        self.trig_rest[5] = self.joy.get_axis(5)
        self.trig_rest[2] = self.joy.get_axis(2)
        self.joystick_on = True
        self.set_status(">>> JOYSTICK - manual control")
        self.add_log(f"joystick ON  (amp={self.amp})")

    def _stop_joystick(self):
        self.joystick_on = False
        for _ in range(5):
            try:
                self.sock.sendto(neutral_packet(self.current_light()), self.thr_addr)
            except Exception:
                pass
        self.set_status("JOYSTICK off - neutral sent")
        self.add_log("joystick OFF")

    def _trigger_amount(self, axis):
        """0.0 (released) .. 1.0 (fully pressed), calibrated to this pad's rest value."""
        rest = self.trig_rest.get(axis, -1.0)
        span = 1.0 - rest
        if abs(span) < 1e-6:
            return 0.0
        amt = max(0.0, min(1.0, (self.joy.get_axis(axis) - rest) / span))
        return 0.0 if amt < 0.05 else amt

    def read_joystick(self):
        """Poll the pad -> (surge, strafe, heave, yaw), each clamped to [-1, 1].
        surge  = LEFT_STICK_Y  (axis 1), up  -> forward
        strafe = RIGHT_STICK_Y (axis 3), +1  -> right, -1 -> left
        heave  = RT (axis 5) up  MINUS  RIGHT_STICK_X (axis 2) pushed-left down
        yaw    = XBOX button (5) right, BACK button (4) left
        """

        def dz(v):
            return 0.0 if abs(v) < JOY_DEADZONE else v

        surge = dz(-self.joy.get_axis(1))  # LEFT_STICK_Y up -> forward
        strafe = dz(self.joy.get_axis(3))  # RIGHT_STICK_Y +1 right, -1 left
        heave_up = self._trigger_amount(5)  # RT trigger
        # axis 2 rests at -1 (it's a trigger on this pad), so read it as a
        # calibrated 0..1 press like RT instead of negating a stick value.
        heave_down = self._trigger_amount(2)
        heave = heave_up - heave_down
        # --- ALT heave (both directions on R-stick X, no RT): replace the three
        #     lines above with:  heave = dz(-self.joy.get_axis(2))   (X-right up)
        yaw = 0.0
        if self.joy.get_button(5):  # XBOX_BUTTON -> yaw right
            yaw += 1.0
        if self.joy.get_button(4):  # BACK       -> yaw left
            yaw -= 1.0
        return clamp(surge), clamp(strafe), clamp(heave), clamp(yaw)

    def send_joystick(self):
        """One control cycle: read the pad, mix, send a thruster packet."""
        surge, strafe, heave, yaw = self.read_joystick()
        heave = self.guard_heave(heave)  # depth band overrides the stick
        self.joy_cmd = (surge, strafe, heave, yaw)
        thr = mix(surge, strafe, heave, yaw)
        try:
            self.sock.sendto(
                thruster_packet(thr, self.amp, self.current_light()), self.thr_addr
            )
        except Exception:
            pass

    def set_dur(self, d):
        self.duration = max(1, min(15, self.duration + d))

    # ---- AMP text box ----
    def amp_editable(self):
        """Editable except during a thruster test. The autonomy worker reads AMP
        every loop, so it can be tuned live during a hunt."""
        return not self.running

    def commit_amp(self):
        """Parse the typed text, clamp to AMP_MIN..AMP_MAX, and deactivate the box.
        Empty or non-numeric input falls back to the current amp."""
        try:
            v = int(self.amp_text)
        except (ValueError, TypeError):
            v = self.amp
        v = max(AMP_MIN, min(AMP_MAX, v))
        self.amp = v
        self.amp_text = str(v)
        self.amp_active = False

    # ---- depth safety envelope ----
    def effective_depth(self):
        """Tared depth in metres, positive == deeper (always, regardless of the
        raw sensor's sign). None if there's no fresh depth data."""
        sv = self.sensors.get()
        if sv is None:
            return None
        d = sv[0] - self.depth_zero
        return d if DEPTH_INCREASES_DOWN else -d

    def guard_heave(self, heave):
        """Clamp/override a heave command so the sub can't leave the depth band.
        heave sign: + == up == shallower, - == down == deeper. Near a limit we
        stop allowing "further past"; once over it we actively push back. With no
        depth data the command passes through unchanged (guard can't act)."""
        d = self.effective_depth()
        if d is None:
            return heave
        if d >= self.depth_max:  # too deep -> climb back
            return max(heave, DEPTH_RECOVER)
        if d >= self.depth_max - DEPTH_MARGIN:  # near deep limit -> no further down
            return min(heave, 0.0)
        if d <= self.depth_min:  # too shallow -> dive back
            return min(heave, -DEPTH_RECOVER)
        if d <= self.depth_min + DEPTH_MARGIN:  # near shallow limit -> no further up
            return max(heave, 0.0)
        return heave

    def zero_depth(self):
        """Snapshot the current raw depth as the new 0 (press at the surface)."""
        sv = self.sensors.get()
        if sv is None:
            self.set_status("ZERO failed - no depth data")
            return
        self.depth_zero = sv[0]
        self.set_status(f"depth zeroed (raw {sv[0] * 100:+.1f} cm -> 0)")
        self.add_log("depth zeroed at surface")

    # ---- depth hold (independent PI mode) ----
    def toggle_depth_hold(self):
        if self.depth_hold:
            self._stop_depth_hold()
        elif not self.running and not self.autonomous and not self.joystick_on:
            self._start_depth_hold()

    def _start_depth_hold(self):
        if self.effective_depth() is None:
            self.set_status("DEPTH HOLD needs depth data - not started")
            return
        if self.hold_target_active:
            self.commit_hold_target()
        self._hold_i = 0.0  # fresh integrator each engage
        self._hold_last_t = time.time()
        self.depth_hold = True
        self.abort = False
        self.set_status(f">>> DEPTH HOLD @ {self.hold_target:.2f} m")
        self.add_log(f"depth hold ON  target {self.hold_target:.2f} m")

    def _stop_depth_hold(self):
        self.depth_hold = False
        for _ in range(5):
            try:
                self.sock.sendto(neutral_packet(self.current_light()), self.thr_addr)
            except Exception:
                pass
        self.set_status("DEPTH HOLD off - neutral sent")
        self.add_log("depth hold OFF")

    def step_depth_hold(self, now):
        """One PI step: hold effective_depth() at hold_target and send the packet.
        heave sign: + == up (shallower), - == down (deeper). err = depth - target,
        so err > 0 (too deep) -> positive heave (up); the integral trims the steady
        down-bias this positively-buoyant sub needs. If depth data drops out we send
        neutral (the sub then drifts UP, away from the floor) and reset the integrator."""
        d = self.effective_depth()
        if d is None:
            self._hold_i = 0.0
            try:
                self.sock.sendto(neutral_packet(self.current_light()), self.thr_addr)
            except Exception:
                pass
            self.set_status("DEPTH HOLD - no depth data, neutral")
            return
        dt = (now - self._hold_last_t) if self._hold_last_t else 0.0
        self._hold_last_t = now
        err = d - self.hold_target
        # integrate only outside the deadband, so sensor noise / mm-drift can't make
        # it hunt; the integrator HOLDS its value inside the band (keeps the trim).
        if dt > 0.0 and abs(err) >= HOLD_DEADBAND:
            self._hold_i += err * dt
            if HOLD_KI > 0.0:  # clamp the integrator to its heave budget (anti-windup)
                i_cap = HOLD_I_LIMIT / HOLD_KI
                self._hold_i = max(-i_cap, min(i_cap, self._hold_i))
        i_term = max(-HOLD_I_LIMIT, min(HOLD_I_LIMIT, HOLD_KI * self._hold_i))
        heave = clamp(HOLD_KP * err + i_term)
        thr = mix(0.0, 0.0, heave, 0.0)  # depth only -- no surge/strafe/yaw
        try:
            self.sock.sendto(
                thruster_packet(thr, self.amp, self.current_light()), self.thr_addr
            )
        except Exception:
            pass
        with self.lock:
            self.auto_cmd = (0.0, 0.0, heave, 0.0)  # reuse for the on-screen readout

    def hold_target_editable(self):
        """The hold target is this mode's control, so it stays editable even while
        holding (retype to fly to a new depth); locked only during a test."""
        return not self.running

    def commit_hold_target(self):
        v = self._parse_float(self.hold_target_text, self.hold_target)
        v = max(-2.0, min(9.9, v))
        self.hold_target = v
        self.hold_target_text = f"{v:.2f}"
        self.hold_target_active = False

    def depth_editable(self):
        """Min/max are read live by the guard, so they're editable anytime except
        during a running test OR an active hold (hold ignores the guard band)."""
        return not self.running and not self.depth_hold

    @staticmethod
    def _parse_float(text, fallback):
        try:
            return float(text)
        except (ValueError, TypeError):
            return fallback

    def commit_depth_min(self):
        v = self._parse_float(self.depth_min_text, self.depth_min)
        v = max(-2.0, min(9.9, v))
        if v > self.depth_max:  # keep min <= max
            v = self.depth_max
        self.depth_min = v
        self.depth_min_text = f"{v:.2f}"
        self.depth_min_active = False

    def commit_depth_max(self):
        v = self._parse_float(self.depth_max_text, self.depth_max)
        v = max(-2.0, min(9.9, v))
        if v < self.depth_min:  # keep max >= min
            v = self.depth_min
        self.depth_max = v
        self.depth_max_text = f"{v:.2f}"
        self.depth_max_active = False

    def _commit_text_boxes(self, keep=None):
        """Commit every active click-to-type box except the one named by `keep`."""
        if self.amp_active and keep != "amp":
            self.commit_amp()
        if self.target_active and keep != "target":
            self.commit_target()
        if self.depth_min_active and keep != "min":
            self.commit_depth_min()
        if self.depth_max_active and keep != "max":
            self.commit_depth_max()
        if self.hold_target_active and keep != "hold":
            self.commit_hold_target()

    def _draw_num_box(self, s, rect, active, editable, text, value_str):
        """Small text-box renderer shared by the depth min/max fields (same look
        as the amp/target boxes, at the 18 px button font)."""
        if active:
            bg, bd = (60, 70, 90), (255, 230, 120)
        elif editable:
            bg, bd = (45, 47, 55), (150, 150, 160)
        else:
            bg, bd = (40, 40, 45), (80, 80, 85)
        pygame.draw.rect(s, bg, rect, border_radius=6)
        pygame.draw.rect(s, bd, rect, width=2, border_radius=6)
        disp = text if active else value_str
        if active and int(time.time() * 2) % 2 == 0:
            disp += "|"
        col = (255, 255, 255) if editable else (120, 120, 120)
        t = self.f_btn.render(disp, True, col)
        s.blit(t, t.get_rect(midleft=(rect.x + 6, rect.centery)))

    def set_status(self, s):
        with self.lock:
            self.status = s

    def add_log(self, s):
        with self.lock:
            self.log.append(s)
            self.log = self.log[-5:]

    # thruster test run
    def start(self, motion, sign):
        if self.running:
            return
        self.running = True
        self.abort = False
        threading.Thread(target=self._worker, args=(motion, sign), daemon=True).start()

    def stop(self):
        self.abort = True
        self.autonomous = False  # kills the autonomy loop too
        self.joystick_on = False  # and manual gamepad control
        self.depth_hold = False  # and depth hold
        for _ in range(5):
            self.sock.sendto(neutral_packet(self.current_light()), self.thr_addr)
        self.set_status("STOP - neutral sent")

    def _worker(self, motion, sign):
        cmd = {"surge": 0.0, "strafe": 0.0, "heave": 0.0, "yaw": 0.0}
        cmd[motion] = float(sign)
        thr = mix(cmd["surge"], cmd["strafe"], cmd["heave"], cmd["yaw"])
        label = f"{motion} {'+' if sign > 0 else '-'}"
        self.add_log(f"{label}  amp={self.amp}  {self.duration}s")
        recording = self.capture
        if recording:
            fname = f"capture_{motion}_{'+' if sign > 0 else '-'}_{time.strftime('%H%M%S')}.csv"
            self.sensors.start_record(fname)
            self.add_log(f"recording -> {fname}")
        self.sock.sendto(neutral_packet(self.current_light()), self.thr_addr)
        dt = 1.0 / self.rate
        for i in range(self.countdown, 0, -1):
            if self.abort:
                break
            self.set_status(f"Starting in {i}...")
            time.sleep(1.0)
        start_sensor = self.sensors.get() if not self.abort else None
        if not self.abort:
            self.set_status(f">>> GO  ({label})")
            t_next = time.time()
            t_end = t_next + self.duration
            while time.time() < t_end and not self.abort:
                # rebuild each send so LIGHT / TAG FLASH stay live during the test,
                # and re-guard heave so the test also can't cross the depth band
                gthr = mix(
                    cmd["surge"],
                    cmd["strafe"],
                    self.guard_heave(cmd["heave"]),
                    cmd["yaw"],
                )
                self.sock.sendto(
                    thruster_packet(gthr, self.amp, self.current_light()),
                    self.thr_addr,
                )
                t_next += dt
                d = t_next - time.time()
                if d > 0:
                    time.sleep(d)
                else:
                    t_next = time.time()
            self.set_status("ABORTED" if self.abort else ">>> STOP - settling...")
        # sensor state at thrust cutoff (end of powered phase), captured BEFORE the
        # coast so we can split powered (start->cutoff) from glide (cutoff->rest)
        mid_sensor = self.sensors.get() if not self.abort else None
        for _ in range(int(0.5 / dt) + 1):
            self.sock.sendto(neutral_packet(self.current_light()), self.thr_addr)
            time.sleep(dt)
        # let the sub finish gliding, then report powered vs glide for both DOFs
        if not self.abort:
            time.sleep(2.0)
            end_sensor = self.sensors.get()
            if start_sensor and mid_sensor and end_sensor:
                # depth in metres (matches the sim heave box); yaw in degrees
                pow_depth = abs(mid_sensor[0] - start_sensor[0])  # start -> cutoff
                gl_depth = abs(end_sensor[0] - mid_sensor[0])  # cutoff -> rest
                pow_yaw = abs(mid_sensor[1] - start_sensor[1])
                gl_yaw = abs(end_sensor[1] - mid_sensor[1])
                self.add_log(
                    f"depth  P {pow_depth:.3f} m ({pow_depth * 100:.1f} cm)  "
                    f"G {gl_depth:.3f} m ({gl_depth * 100:.1f} cm)"
                )
                self.add_log(f"yaw    P {pow_yaw:.1f} deg  G {gl_yaw:.1f} deg")
                self.set_status(
                    f"P/G depth {pow_depth:.3f}/{gl_depth:.3f}m  "
                    f"yaw {pow_yaw:.0f}/{gl_yaw:.0f}deg"
                )
            else:
                self.set_status(">>> STOP - measure start to rest (no sensor data)")
        if recording:
            self.sensors.stop_record()
        self.running = False

    # main loop
    def run(self):
        clock = pygame.time.Clock()
        last_keepalive = 0.0
        while True:
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    self.shutdown()
                    return
                if e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                    # Text boxes: click inside to focus, click outside to commit.
                    if self.amp_rect.collidepoint(e.pos) and self.amp_editable():
                        self._commit_text_boxes(keep="amp")
                        self.amp_active = True
                        continue
                    if self.target_rect.collidepoint(e.pos) and self.target_editable():
                        self._commit_text_boxes(keep="target")
                        self.target_active = True
                        continue
                    if (
                        self.depth_min_rect.collidepoint(e.pos)
                        and self.depth_editable()
                    ):
                        self._commit_text_boxes(keep="min")
                        self.depth_min_active = True
                        continue
                    if (
                        self.depth_max_rect.collidepoint(e.pos)
                        and self.depth_editable()
                    ):
                        self._commit_text_boxes(keep="max")
                        self.depth_max_active = True
                        continue
                    if (
                        self.hold_target_rect.collidepoint(e.pos)
                        and self.hold_target_editable()
                    ):
                        self._commit_text_boxes(keep="hold")
                        self.hold_target_active = True
                        continue
                    self._commit_text_boxes()  # clicked elsewhere -> commit any box
                    # Tune-panel sliders take the click before buttons; they don't
                    # overlap (sliders sit in the scroll band, buttons in header).
                    if self._panel_slider_hit(e.pos):
                        continue
                    for b in self.buttons:
                        if b.click(e.pos):
                            break
                if e.type == pygame.MOUSEMOTION and self.slider_drag is not None:
                    self._drag_slider(e.pos)
                if e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                    self.slider_drag = None  # release; debounced write handles save
                if e.type == pygame.MOUSEWHEEL and self.panel_open:
                    mx, _my = pygame.mouse.get_pos()
                    if mx >= self.PANEL_X:  # only scroll when over the panel
                        self.panel_scroll -= e.y * 30
                        self.panel_scroll = max(
                            0, min(self._scroll_max(), self.panel_scroll)
                        )
                if e.type == pygame.KEYDOWN and self.amp_active:
                    if e.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                        self.commit_amp()
                    elif e.key == pygame.K_ESCAPE:
                        # discard edits, restore committed value
                        self.amp_text = str(self.amp)
                        self.amp_active = False
                    elif e.key == pygame.K_BACKSPACE:
                        self.amp_text = self.amp_text[:-1]
                    elif e.unicode.isdigit() and len(self.amp_text) < 3:
                        self.amp_text += e.unicode
                if e.type == pygame.KEYDOWN and self.target_active:
                    if e.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                        self.commit_target()
                    elif e.key == pygame.K_ESCAPE:
                        self.target_text = str(self.tag_target)
                        self.target_active = False
                    elif e.key == pygame.K_BACKSPACE:
                        self.target_text = self.target_text[:-1]
                    elif e.unicode.isdigit() and len(self.target_text) < 2:
                        self.target_text += e.unicode
                if e.type == pygame.KEYDOWN and self.depth_min_active:
                    if e.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                        self.commit_depth_min()
                    elif e.key == pygame.K_ESCAPE:
                        self.depth_min_text = f"{self.depth_min:.2f}"
                        self.depth_min_active = False
                    elif e.key == pygame.K_BACKSPACE:
                        self.depth_min_text = self.depth_min_text[:-1]
                    elif e.unicode in "0123456789.-" and len(self.depth_min_text) < 6:
                        self.depth_min_text += e.unicode
                if e.type == pygame.KEYDOWN and self.depth_max_active:
                    if e.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                        self.commit_depth_max()
                    elif e.key == pygame.K_ESCAPE:
                        self.depth_max_text = f"{self.depth_max:.2f}"
                        self.depth_max_active = False
                    elif e.key == pygame.K_BACKSPACE:
                        self.depth_max_text = self.depth_max_text[:-1]
                    elif e.unicode in "0123456789.-" and len(self.depth_max_text) < 6:
                        self.depth_max_text += e.unicode
                if e.type == pygame.KEYDOWN and self.hold_target_active:
                    if e.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                        self.commit_hold_target()
                    elif e.key == pygame.K_ESCAPE:
                        self.hold_target_text = f"{self.hold_target:.2f}"
                        self.hold_target_active = False
                    elif e.key == pygame.K_BACKSPACE:
                        self.hold_target_text = self.hold_target_text[:-1]
                    elif e.unicode in "0123456789.-" and len(self.hold_target_text) < 6:
                        self.hold_target_text += e.unicode
                # ---- gamepad: D-pad steps amp, X/B toggle light/tag-flash ----
                if (
                    e.type == pygame.JOYHATMOTION
                    and self.joystick_on
                    and self.amp_editable()
                ):
                    if e.value[1] > 0:  # D-pad up
                        self.amp = min(AMP_MAX, self.amp + 100)
                        self.amp_text = str(self.amp)
                    elif e.value[1] < 0:  # D-pad down
                        self.amp = max(AMP_MIN, self.amp - 100)
                        self.amp_text = str(self.amp)
                if e.type == pygame.JOYBUTTONDOWN and self.joystick_on:
                    if e.button == 2:  # X -> light toggle
                        self.toggle_light()
                    elif e.button == 1:  # B -> tag flash toggle
                        self.toggle_tag_flash()
            # Collect unique AprilTags while autonomous. They don't have to be
            # seen together -- each new ID is remembered until RESET TAGS.
            # Reaching tag_target flashes the lights and hands off to the pad.
            if self.autonomous and not self.mission_complete:
                ids = self.video.get_tag_ids()
                if ids:
                    before = len(self.collected_tags)
                    self.collected_tags.update(ids)
                    if len(self.collected_tags) != before:
                        self.add_log(
                            f"unique tags {len(self.collected_tags)}/{self.tag_target}: "
                            f"{sorted(self.collected_tags)}"
                        )
                    if len(self.collected_tags) >= self.tag_target:
                        self._complete_mission()
            # idle keep-alive / manual drive: when NOT running a test and NOT
            # autonomous, either drive from the pad (which also serves as the
            # server keep-alive) or send neutral packets carrying the current
            # light value. Keeps LIGHT / TAG FLASH live and satisfies the
            # server's 0.5s watchdog. ~20-30 Hz is plenty.
            now = time.time()
            if not self.running and not self.autonomous:
                if self.depth_hold:
                    self.step_depth_hold(now)  # PI loop drives + keeps the link alive
                    last_keepalive = now
                elif self.joystick_on and self.joy is not None:
                    self.send_joystick()
                    last_keepalive = now
                elif now - last_keepalive >= 0.05:
                    try:
                        self.sock.sendto(
                            neutral_packet(self.current_light()), self.thr_addr
                        )
                    except Exception:
                        pass
                    last_keepalive = now
            # debounced gains write: collapse a slider drag into one file write
            # ~150 ms after the last edit, so the brain hot-reloads live but we
            # don't hammer the disk every frame.
            if self._gains_dirty and time.time() - self._gains_last_edit > 0.15:
                self._save_gains()
            self.draw()
            clock.tick(30)

    def shutdown(self):
        self.abort = True
        self.autonomous = False
        self.joystick_on = False
        self.depth_hold = False
        try:
            self.sock.sendto(neutral_packet(), self.thr_addr)
        except Exception:
            pass
        self.video.stop_event.set()
        self.sensors.stop_event.set()
        time.sleep(0.1)
        pygame.quit()

    def draw(self):
        s = self.screen
        s.fill((30, 32, 38))
        s.blit(
            self.f_title.render(
                f"ROV Client ({self.mode.upper()})", True, (240, 240, 240)
            ),
            (20, 16),
        )
        # captions + values for duration stepper
        s.blit(self.f_small.render("Duration", True, (180, 180, 180)), (20, 70))
        s.blit(self.f_small.render("AMP (0-400)", True, (180, 180, 180)), (200, 70))
        s.blit(
            self.f_status.render(f"{self.duration}s", True, (255, 255, 255)), (66, 92)
        )
        # AMP text box
        editable = self.amp_editable()
        if self.amp_active:
            box_bg, border = (60, 70, 90), (255, 230, 120)
        elif editable:
            box_bg, border = (45, 47, 55), (150, 150, 160)
        else:
            box_bg, border = (40, 40, 45), (80, 80, 85)  # greyed while running
        pygame.draw.rect(s, box_bg, self.amp_rect, border_radius=6)
        pygame.draw.rect(s, border, self.amp_rect, width=2, border_radius=6)
        disp = self.amp_text if self.amp_active else str(self.amp)
        if self.amp_active and int(time.time() * 2) % 2 == 0:
            disp += "|"  # blinking caret
        txt_col = (255, 255, 255) if editable else (120, 120, 120)
        at = self.f_status.render(disp, True, txt_col)
        s.blit(at, at.get_rect(midleft=(self.amp_rect.x + 8, self.amp_rect.centery)))
        for b in self.buttons:
            b.draw(s, self.f_btn)
        with self.lock:
            status = self.status
            log = list(self.log)
        s.blit(self.f_status.render(status, True, (255, 230, 120)), (20, self.status_y))
        for i, line in enumerate(log):
            s.blit(
                self.f_small.render(line, True, (170, 170, 175)),
                (20, self.status_y + 30 + i * 20),
            )
        # video panel
        pygame.draw.rect(s, (0, 0, 0), self.vid_rect)
        pygame.draw.rect(s, (90, 90, 100), self.vid_rect, width=2)
        frame = self.video.get_frame()
        if frame is not None:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w = rgb.shape[:2]
            surf = pygame.image.frombuffer(rgb.tobytes(), (w, h), "RGB")
            scale = min(self.vid_rect.w / w, self.vid_rect.h / h)
            surf = pygame.transform.smoothscale(surf, (int(w * scale), int(h * scale)))
            s.blit(surf, surf.get_rect(center=self.vid_rect.center))
        else:
            t = self.f_status.render("NO VIDEO", True, (120, 120, 120))
            s.blit(t, t.get_rect(center=self.vid_rect.center))
        # perception latency readout (bottom-left of the video): how stale the
        # freshest YOLO box is by the time a 30 Hz consumer reads it -- i.e. the
        # delay #2/#3 would remove -- plus the rate new boxes arrive. render.py's
        # on-screen number is inference-only (~10 ms); total ~= that + box lag.
        if self.video.yolo_on:
            lag_ms, det_hz = self.video.get_box_lag()
            self._box_lag_ms = (
                lag_ms
                if self._box_lag_ms == 0.0
                else 0.85 * self._box_lag_ms + 0.15 * lag_ms
            )
            # While autonomous, show the control loop's own box age (what #3 drives
            # down); otherwise show the 30 Hz display proxy. det is always shown.
            if self.autonomous:
                label, shown = "ctrl lag", self._ctrl_lag_ms
            else:
                label, shown = "box lag", self._box_lag_ms
            lag_txt = f"{label} {shown:5.1f} ms   det {det_hz:4.1f} Hz"
            s.blit(
                self.f_small.render(lag_txt, True, (150, 200, 220)),
                (self.vid_rect.x + 8, self.vid_rect.bottom - 24),
            )
        if self.video.rec.on:
            s.blit(
                self.f_small.render("\u25cf REC", True, (255, 80, 80)),
                (self.vid_rect.x + 8, self.vid_rect.y + 8),
            )
        if self.depth_hold:
            hv = self.auto_cmd[2]
            d = self.effective_depth()
            s.blit(
                self.f_status.render("DEPTH HOLD", True, (120, 220, 210)),
                (self.vid_rect.x + 8, self.vid_rect.y + 26),
            )
            if d is not None:
                sub = f"target {self.hold_target:.2f}  now {d:.2f} m  err {d - self.hold_target:+.2f}  heave {hv:+.2f}"
            else:
                sub = "no depth data - holding neutral"
            s.blit(
                self.f_small.render(sub, True, (150, 200, 220)),
                (self.vid_rect.x + 8, self.vid_rect.y + 52),
            )
        elif self.autonomous:
            s.blit(
                self.f_status.render(self.auto_state, True, (120, 230, 150)),
                (self.vid_rect.x + 8, self.vid_rect.y + 26),
            )
        elif self.joystick_on:
            su, st_, hv, yw = self.joy_cmd
            s.blit(
                self.f_status.render("MANUAL (pad)", True, (120, 200, 230)),
                (self.vid_rect.x + 8, self.vid_rect.y + 26),
            )
            s.blit(
                self.f_small.render(
                    f"surge {su:+.2f}  strafe {st_:+.2f}  "
                    f"heave {hv:+.2f}  yaw {yw:+.2f}",
                    True,
                    (150, 200, 220),
                ),
                (self.vid_rect.x + 8, self.vid_rect.y + 52),
            )
        # AprilTag readout
        ids = self.video.get_tag_ids()
        tag_txt = "AprilTag IDs: " + (", ".join(str(i) for i in ids) if ids else "none")
        s.blit(
            self.f_status.render(
                tag_txt, True, (120, 230, 120) if ids else (150, 150, 150)
            ),
            (565, 496),
        )
        # sensor readout (depth = heave, yaw). Depth is shown TARED (from ZERO)
        # and coloured by band: red at/over a limit, amber within the margin.
        sv = self.sensors.get()
        d = self.effective_depth()
        if sv is not None and d is not None:
            sensor_txt = f"Depth: {d * 100:+.1f} cm    Yaw: {sv[1]:+.1f} deg"
            if d >= self.depth_max or d <= self.depth_min:
                color = (240, 120, 120)  # out of band -> guard actively pushing
            elif (
                d >= self.depth_max - DEPTH_MARGIN or d <= self.depth_min + DEPTH_MARGIN
            ):
                color = (240, 200, 120)  # in the margin -> no further past allowed
            else:
                color = (120, 200, 230)
        else:
            sensor_txt = "Sensors: no data (depth guard inactive)"
            color = (150, 150, 150)
        s.blit(self.f_status.render(sensor_txt, True, color), (565, 526))
        # depth-guard controls (bottom strip): caption + min/max boxes. The
        # ZERO DEPTH button lives in self.buttons and is drawn with the others.
        d_edit = self.depth_editable()
        s.blit(
            self.f_small.render(
                "Depth (m) - min/max guard band, or HOLD to a set depth",
                True,
                (180, 180, 180),
            ),
            (372, 662),
        )
        s.blit(self.f_small.render("min", True, (180, 180, 180)), (372, 686))
        self._draw_num_box(
            s,
            self.depth_min_rect,
            self.depth_min_active,
            d_edit,
            self.depth_min_text,
            f"{self.depth_min:.2f}",
        )
        s.blit(self.f_small.render("max", True, (180, 180, 180)), (476, 686))
        self._draw_num_box(
            s,
            self.depth_max_rect,
            self.depth_max_active,
            d_edit,
            self.depth_max_text,
            f"{self.depth_max:.2f}",
        )
        s.blit(self.f_small.render("hold", True, (180, 180, 180)), (578, 686))
        self._draw_num_box(
            s,
            self.hold_target_rect,
            self.hold_target_active,
            self.hold_target_editable(),
            self.hold_target_text,
            f"{self.hold_target:.2f}",
        )
        # tag-collection: caption + target text box + progress
        s.blit(self.f_small.render("Tags to finish", True, (180, 180, 180)), (372, 534))
        t_edit = self.target_editable()
        if self.target_active:
            tbg, tbd = (60, 70, 90), (255, 230, 120)
        elif t_edit:
            tbg, tbd = (45, 47, 55), (150, 150, 160)
        else:
            tbg, tbd = (40, 40, 45), (80, 80, 85)
        pygame.draw.rect(s, tbg, self.target_rect, border_radius=6)
        pygame.draw.rect(s, tbd, self.target_rect, width=2, border_radius=6)
        tdisp = self.target_text if self.target_active else str(self.tag_target)
        if self.target_active and int(time.time() * 2) % 2 == 0:
            tdisp += "|"
        tcol = (255, 255, 255) if t_edit else (120, 120, 120)
        tt = self.f_status.render(tdisp, True, tcol)
        s.blit(
            tt, tt.get_rect(midleft=(self.target_rect.x + 8, self.target_rect.centery))
        )
        n_got = len(self.collected_tags)
        if self.mission_complete:
            prog_txt, prog_col = f"COMPLETE  {n_got}/{self.tag_target}", (120, 230, 150)
        else:
            prog_txt, prog_col = f"Collected {n_got}/{self.tag_target}", (170, 170, 175)
        s.blit(self.f_small.render(prog_txt, True, prog_col), (372, 596))
        # autonomy readout (state + the four commands going into the mixer).
        # Placed to the RIGHT of the AUTONOMOUS button (which spans x=370..620,
        # y=614..660) and stacked on two lines so the full command set fits
        # inside the window without running off the right edge.
        if self.autonomous:
            su, st_, hv, yw = self.auto_cmd
            auto_col = (120, 230, 150)
            s.blit(
                self.f_small.render(
                    f"AUTO {self.auto_state}:  surge {su:+.2f}  strafe {st_:+.2f}",
                    True,
                    auto_col,
                ),
                (640, 620),
            )
            s.blit(
                self.f_small.render(f"heave {hv:+.2f}  yaw {yw:+.2f}", True, auto_col),
                (640, 640),
            )
        else:
            if not HAVE_STRATEGY:
                auto_txt = f"AUTO: {_STRATEGY_NAME} not found"
                auto_col = (200, 140, 128)
            elif not self.video.yolo_on:
                auto_txt = "AUTO: needs YOLO (start with --weights)"
                auto_col = (150, 150, 150)
            else:
                auto_txt = f"AUTO: off ({_STRATEGY_NAME})"
                auto_col = (150, 150, 150)
            s.blit(self.f_small.render(auto_txt, True, auto_col), (640, 630))
        if self.panel_open:
            self.draw_panel(s)
        pygame.display.flip()

    def draw_panel(self, s):
        """Docked slider panel on the right. Header + Reload/Reset buttons are
        fixed; the slider list scrolls (clipped to the band between SCROLL_TOP and
        SCROLL_BOT); a footer shows the hovered slider's description + save status."""
        px = self.PANEL_X
        # background + left divider
        pygame.draw.rect(s, (24, 26, 31), (px, 0, self.PANEL_W, self.H))
        pygame.draw.line(s, (70, 72, 80), (px, 0), (px, self.H), 2)
        s.blit(
            self.f_title.render("Strategy gains", True, (235, 235, 235)), (px + 14, 14)
        )
        # header buttons (Reload/Reset) are in self.buttons and drawn already.
        # which slider is the mouse hovering (for footer + handle highlight)?
        mx, my = pygame.mouse.get_pos()
        hover = None
        # clip the scroll list so rows can't spill over the header/footer
        old_clip = s.get_clip()
        s.set_clip(
            pygame.Rect(
                px, self.SCROLL_TOP, self.PANEL_W, self.SCROLL_BOT - self.SCROLL_TOP
            )
        )
        for key, label, lo, hi, res, desc, tx, tw, cy in self._panel_rows():
            ry = self._row_screen_y(cy)  # row top on screen
            ty = ry + 26  # track centre
            if ty + 20 < self.SCROLL_TOP or ry > self.SCROLL_BOT:
                continue  # fully scrolled out
            v = self.gains.get(key, 0.0)
            # label + value
            s.blit(self.f_small.render(label, True, (200, 200, 205)), (tx, ry))
            vt = self.f_small.render(fmt_gain(key, v), True, (255, 230, 120))
            s.blit(vt, vt.get_rect(topright=(tx + tw, ry)))
            # track
            pygame.draw.line(s, (70, 72, 80), (tx, ty), (tx + tw, ty), 4)
            # handle
            frac = 0.0 if hi == lo else (v - lo) / (hi - lo)
            frac = max(0.0, min(1.0, frac))
            hx = int(tx + frac * tw)
            active = key == self.slider_drag
            near = tx - 8 <= mx <= tx + tw + 8 and ty - 12 <= my <= ty + 12
            if near and self.SCROLL_TOP <= ty <= self.SCROLL_BOT:
                hover = (key, desc)
            col = (
                (255, 230, 120)
                if active
                else ((150, 200, 230) if near else (120, 150, 180))
            )
            pygame.draw.circle(s, col, (hx, ty), 7)
        s.set_clip(old_clip)
        # scrollbar hint (only if the list overflows)
        smax = self._scroll_max()
        if smax > 0:
            band_h = self.SCROLL_BOT - self.SCROLL_TOP
            frac = self.panel_scroll / smax
            bar_h = max(24, int(band_h * band_h / (band_h + smax)))
            bar_y = int(self.SCROLL_TOP + frac * (band_h - bar_h))
            pygame.draw.rect(
                s,
                (90, 92, 100),
                (px + self.PANEL_W - 6, bar_y, 4, bar_h),
                border_radius=2,
            )
        # footer: hovered slider's description (wrapped), then save status
        fy = self.SCROLL_BOT + 6
        if hover is not None:
            for line in self._wrap(hover[1], self.PANEL_W - 28):
                s.blit(self.f_small.render(line, True, (150, 150, 155)), (px + 14, fy))
                fy += 18
        stat_col = (255, 180, 120) if self._gains_dirty else (120, 200, 150)
        s.blit(
            self.f_small.render(self.panel_status, True, stat_col),
            (px + 14, self.H - 24),
        )

    def _wrap(self, text, width_px):
        """Greedy word-wrap for the footer description, capped at 3 lines."""
        words = text.split()
        lines, cur = [], ""
        for w in words:
            trial = (cur + " " + w).strip()
            if self.f_small.size(trial)[0] <= width_px:
                cur = trial
            else:
                if cur:
                    lines.append(cur)
                cur = w
            if len(lines) == 3:
                break
        if cur and len(lines) < 3:
            lines.append(cur)
        return lines


def main():
    ap = argparse.ArgumentParser(
        description="Combined topside ROV client (thrusters + video)."
    )
    ap.add_argument("--wifi", action="store_true")
    ap.add_argument(
        "--weights", default=None, help="YOLOv26 weights to enable detection"
    )
    ap.add_argument("--conf", type=float, default=0.6)
    ap.add_argument("--yolo-interval", type=int, default=3)
    ap.add_argument(
        "--strategy",
        default="new_strategy_full",
        help="brain module to load (module name, no .py). "
        "e.g. strategy_full or aqua_strategy",
    )
    args = ap.parse_args()

    # Load the chosen brain and publish it to the module globals App reads.
    global Strategy, BoundingBox, HAVE_STRATEGY, _STRATEGY_ERR, _STRATEGY_NAME
    _STRATEGY_NAME = args.strategy
    Strategy, BoundingBox, HAVE_STRATEGY, _STRATEGY_ERR = load_strategy(args.strategy)

    mode = "wifi" if args.wifi else "lan"
    App(mode, args.weights, args.conf, args.yolo_interval).run()


if __name__ == "__main__":
    mp.freeze_support()
    try:
        mp.set_start_method("spawn")
    except RuntimeError:
        pass  # already set
    main()
