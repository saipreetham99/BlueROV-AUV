#!/usr/bin/env python3
"""
Combined topside ROV client (single pygame window):

  * Thruster test panel (always runs, even with no video): pick a motion, it drives one
    DOF at a fixed level for a fixed duration through the SAME mix/packet as the driver,
    then neutral. STOP aborts. Same engine as pool_test.py / real_test_gui.py.
  * Live video (shown inside the window when the stream is up; "NO VIDEO" otherwise),
    with AprilTag (optional) and YOLO (optional, --weights) overlays and a RECORD button
    that saves the clean stream to mp4.

Mode (lan/wifi) is chosen at launch with --wifi and applies to both video and thrusters.
Reads .rov_server_creds.

  python rov_client.py
  python rov_client.py --wifi --weights best.pt
"""

import argparse
import configparser
import os

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

NEUTRAL = 1500
LIGHT_OFF = 1100


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


def thruster_packet(thr, amp):
    fl, fr, rl, rr, v1, v2 = thr
    return struct.pack(
        "<7H",
        to_pwm(fl, amp),
        to_pwm(fr, amp),
        to_pwm(rl, amp),
        to_pwm(rr, amp),
        to_pwm(v1, amp),
        to_pwm(v2, amp),
        LIGHT_OFF,
    )


def neutral_packet():
    return struct.pack("<7H", *([NEUTRAL] * 6), LIGHT_OFF)


def load_config(mode):
    path = os.path.expanduser(".rov_server_creds")
    cfg = configparser.ConfigParser()
    if not os.path.exists(path) or not cfg.read(path):
        sys.exit(f"✗ ERROR: Config file not found or empty at '{path}'")
    try:
        rov_ip = cfg[mode]["rov_ip"]
        thruster_port = cfg.getint("DEFAULT", "thruster_port")
        video_port = cfg.getint("DEFAULT", "video_port")
        sensor_port = cfg.getint("DEFAULT", "imu_and_depth_port")
    except (KeyError, configparser.NoSectionError) as e:
        sys.exit(f"✗ ERROR: Missing section or key in config: {e}")
    return rov_ip, thruster_port, video_port, sensor_port


def capture_sensors(port, out_path, stop_event):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", port))
    s.settimeout(0.2)
    t0 = time.time()
    with open(out_path, "w") as f:
        f.write("t_seconds,length,hex\n")
        while not stop_event.is_set():
            try:
                data, _ = s.recvfrom(4096)
            except socket.timeout:
                continue
            f.write(f"{time.time() - t0:.4f},{len(data)},{data.hex()}\n")
    s.close()


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


class VideoReceiver(threading.Thread):
    def __init__(self, server_ip, port, weights=None, conf=0.5, yolo_interval=3):
        super().__init__(daemon=True)
        self.server_ip = server_ip
        self.port = port
        self.conf = conf
        self.yolo_interval = max(1, yolo_interval)
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

        self.yolo = None
        if weights:
            from ultralytics import YOLO

            self.yolo = YOLO(weights)
            print(f"[yolo] loaded {weights}")

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
                raw = frame.copy()

                ids = []
                if self.tag_det is not None:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    for tag in self.tag_det.detect(gray):
                        pts = tag.corners.astype(int)
                        for i in range(4):
                            cv2.line(
                                frame,
                                tuple(pts[i]),
                                tuple(pts[(i + 1) % 4]),
                                (0, 255, 0),
                                2,
                            )
                        cx, cy = map(int, tag.center)
                        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                        cv2.putText(
                            frame,
                            f"id {tag.tag_id}",
                            (cx + 8, cy - 8),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 255, 0),
                            2,
                        )
                        ids.append(int(tag.tag_id))

                if self.yolo is not None and rf % self.yolo_interval == 0:
                    res = self.yolo(frame, conf=self.conf, max_det=1, verbose=False)[0]
                    for b in res.boxes:
                        x1, y1, x2, y2 = map(int, b.xyxy[0])
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 180, 0), 2)
                rf += 1

                self.rec.write(raw)
                with self.lock:
                    self.latest = frame
                    self.last_time = time.time()
                    self.tag_ids = ids

            now = time.time()
            for k in [k for k, v in buffers.items() if now - v["ts"] > 0.5]:
                del buffers[k]

        sock.close()
        self.rec.stop()

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
        self.screen = pygame.display.set_mode((self.W, self.H))
        pygame.display.set_caption("ROV Client")
        self.f_title = pygame.font.SysFont("Helvetica", 24, bold=True)
        self.f_btn = pygame.font.SysFont("Helvetica", 18, bold=True)
        self.f_small = pygame.font.SysFont("Helvetica", 15)
        self.f_status = pygame.font.SysFont("Helvetica", 20, bold=True)

        # thruster-test settings/state
        self.duration = 3
        self.amp = 100
        self.capture = False
        self.rate = 50.0
        self.countdown = 3
        self.running = False
        self.abort = False
        self.status = "Ready"
        self.log = []
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # video
        self.video = VideoReceiver(
            self.rov_ip, self.video_port, weights, conf, yolo_interval
        )
        self.video.start()

        # video panel geometry
        self.vid_rect = pygame.Rect(370, 20, 610, 458)

        idle = lambda: not self.running
        self.buttons = []
        self.buttons.append(
            Button((20, 90, 36, 36), "-", lambda: self.set_dur(-1), (80, 80, 90), idle)
        )
        self.buttons.append(
            Button((120, 90, 36, 36), "+", lambda: self.set_dur(+1), (80, 80, 90), idle)
        )
        self.buttons.append(
            Button(
                (200, 90, 36, 36), "-", lambda: self.set_amp(-100), (80, 80, 90), idle
            )
        )
        self.buttons.append(
            Button(
                (300, 90, 36, 36), "+", lambda: self.set_amp(+100), (80, 80, 90), idle
            )
        )
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

    # settings
    def toggle_capture(self):
        self.capture = not self.capture

    def set_dur(self, d):
        self.duration = max(1, min(15, self.duration + d))

    def set_amp(self, d):
        self.amp = max(100, min(400, self.amp + d))

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
        for _ in range(5):
            self.sock.sendto(neutral_packet(), self.thr_addr)
        self.set_status("STOP - neutral sent")

    def _worker(self, motion, sign):
        cmd = {"surge": 0.0, "strafe": 0.0, "heave": 0.0, "yaw": 0.0}
        cmd[motion] = float(sign)
        thr = mix(cmd["surge"], cmd["strafe"], cmd["heave"], cmd["yaw"])
        packet = thruster_packet(thr, self.amp)
        label = f"{motion} {'+' if sign > 0 else '-'}"
        self.add_log(f"{label}  amp={self.amp}  {self.duration}s")

        stop_event = threading.Event()
        cap_thread = None
        if self.capture:
            fname = f"capture_{motion}_{'+' if sign > 0 else '-'}_{time.strftime('%H%M%S')}.csv"
            cap_thread = threading.Thread(
                target=capture_sensors,
                args=(self.sensor_port, fname, stop_event),
                daemon=True,
            )
            cap_thread.start()
            self.add_log(f"recording -> {fname}")

        self.sock.sendto(neutral_packet(), self.thr_addr)
        dt = 1.0 / self.rate

        for i in range(self.countdown, 0, -1):
            if self.abort:
                break
            self.set_status(f"Starting in {i}...")
            time.sleep(1.0)

        if not self.abort:
            self.set_status(f">>> GO  ({label})")
            t_next = time.time()
            t_end = t_next + self.duration
            while time.time() < t_end and not self.abort:
                self.sock.sendto(packet, self.thr_addr)
                t_next += dt
                d = t_next - time.time()
                if d > 0:
                    time.sleep(d)
                else:
                    t_next = time.time()
            self.set_status(
                "ABORTED" if self.abort else ">>> STOP - measure start to rest"
            )

        for _ in range(int(0.5 / dt) + 1):
            self.sock.sendto(neutral_packet(), self.thr_addr)
            time.sleep(dt)
        stop_event.set()
        if cap_thread:
            cap_thread.join(timeout=1.0)
        self.running = False

    # main loop
    def run(self):
        clock = pygame.time.Clock()
        while True:
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    self.shutdown()
                    return
                if e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                    for b in self.buttons:
                        if b.click(e.pos):
                            break
            self.draw()
            clock.tick(30)

    def shutdown(self):
        self.abort = True
        try:
            self.sock.sendto(neutral_packet(), self.thr_addr)
        except Exception:
            pass
        self.video.stop_event.set()
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

        # captions + values for steppers
        s.blit(self.f_small.render("Duration", True, (180, 180, 180)), (20, 70))
        s.blit(self.f_small.render("AMP", True, (180, 180, 180)), (200, 70))
        s.blit(
            self.f_status.render(f"{self.duration}s", True, (255, 255, 255)), (66, 92)
        )
        s.blit(self.f_status.render(f"{self.amp}", True, (255, 255, 255)), (244, 92))

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

        if self.video.rec.on:
            s.blit(
                self.f_small.render("● REC", True, (255, 80, 80)),
                (self.vid_rect.x + 8, self.vid_rect.y + 8),
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

        pygame.display.flip()


def main():
    ap = argparse.ArgumentParser(
        description="Combined topside ROV client (thrusters + video)."
    )
    ap.add_argument("--wifi", action="store_true")
    ap.add_argument(
        "--weights", default=None, help="YOLOv8 weights to enable detection"
    )
    ap.add_argument("--conf", type=float, default=0.5)
    ap.add_argument("--yolo-interval", type=int, default=3)
    args = ap.parse_args()
    mode = "wifi" if args.wifi else "lan"
    App(mode, args.weights, args.conf, args.yolo_interval).run()


if __name__ == "__main__":
    main()
