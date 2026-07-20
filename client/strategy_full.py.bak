"""
strategy_full.py  --  competition-backup brain.

Ported from the archived external_controller.py ("ChaseAndCircleStrategy"). Only
the state machine, the centering math, the grace period, and the search spin were
carried over; the YOLO / camera / dual-feed code isn't needed here because
run_sim.py already hands this class a clean BoundingBox. The same BoundingBox
interface means it doesn't care whether the box comes from Unity's projected
target or from YOLO on the real sub.

Same interface as before (BoundingBox in, four numbers + flash out), so it drops
straight into run_sim.py and onto the real sub with no changes:

    # in run_sim.py:
    from strategy_full import Strategy, BoundingBox

Behaviour:
    IDLE/SEARCHING -> ADVANCING -> ORBITING -> CELEBRATING, with a grace period so
    a one-frame detection dropout doesn't flip the state. With no target in view
    the sub SPINS -- it yaws continuously (a full-circle scan) until a box
    reappears, then locks straight into the chase. The spin turns TOWARD the side
    the target was last seen: exit left -> yaw left, exit right -> yaw right (so it
    follows the target out rather than spinning away from it). Before any target
    has ever been seen it defaults to spinning right.

Two deliberate changes vs the archive, to match the Unity sim's conventions:
    * yaw is NEGATED for steering (sim: +yaw = turn right, so steer toward target).
    * ORBITING heave uses the SAME sign as ADVANCING (the archive negated it,
      which drove the sub the wrong way vertically while orbiting).

Steering gains are SPLIT into yaw_kp (horizontal) and heave_kp (vertical) so the
snappy turn axis and the sluggish depth axis tune independently. The physical
thruster gains (surge/strafe/heave/yaw Gain in the sim) are a separate concern --
tune those to match the real sub, then leave them alone.

Tuning (single source of truth):
    Every knob lives in strategy_gains.json next to this file. It's loaded on
    startup and HOT-RELOADED while running -- edit it (by hand or with the
    tune_gui.py slider panel), save, and the change lands on the next control
    loop. That file rides along with this brain, so the values you tune in the
    sim are exactly what the real sub runs. Missing file or a mid-edit (bad JSON)
    -> the hardcoded defaults below are used and the last good values are kept.
"""

from dataclasses import dataclass
from typing import Tuple
import json
import os

GAINS_FILE = "strategy_gains.json"  # sits next to this file; the one source of truth


@dataclass
class BoundingBox:
    x: float = 0.0
    y: float = 0.0
    width: float = 0.0
    height: float = 0.0

    @property
    def center(self) -> Tuple[float, float]:
        return (self.x + self.width / 2, self.y + self.height / 2)

    @property
    def area(self) -> float:
        return self.width * self.height

    @property
    def is_valid(self) -> bool:
        return self.width > 0 and self.height > 0


class Strategy:
    def __init__(self, camera_width: int = 640, camera_height: int = 480):
        self.center_x = camera_width / 2
        self.center_y = camera_height / 2
        self._full = camera_width * camera_height  # frame area (for the % thresholds)
        self.state = "IDLE"

        # --- defaults (archive values; overridden by strategy_gains.json if present) ---
        self.yaw_kp = 0.0003  # steering strength, horizontal (turn)
        self.heave_kp = 0.0003  # steering strength, vertical (up/down)
        self.advance_surge = 0.8  # forward speed while chasing
        self.orbit_strafe = 0.8  # sideways speed while circling
        self.max_yaw_error_for_strafe = 80.0  # px: must be this centred to strafe

        self.orbit_enter_area = self._full * 0.05  # close enough to orbit
        self.orbit_exit_area = self.orbit_enter_area * 0.75  # drifted away -> re-chase

        self.orbit_to_win_s = 10.0  # orbit this long = success
        self.celebrate_s = 5.0
        self.state_timer = 0.0

        self.grace_s = 0.5  # tolerate brief detection dropouts
        self.lost_timer = 0.0

        # --- search: spin to scan a full circle for the target ---
        self.search_yaw_command = 0.1  # spin rate while searching (magnitude)
        self.search_dir = 1.0  # +1 spin right, -1 spin left; set to last-seen side

        # --- gains file: load once now, hot-reload later ---
        self._gains_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), GAINS_FILE
        )
        self._gains_mtime = None
        self._load_gains()
        try:
            self._gains_mtime = os.path.getmtime(self._gains_path)
            print(f"[strategy] gains loaded from {GAINS_FILE}")
        except OSError:
            print(f"[strategy] {GAINS_FILE} not found -> using built-in defaults")

    # ---- gains file ----
    def _load_gains(self):
        """Read strategy_gains.json and apply it over the current values.
        Missing file or a mid-edit (bad JSON) -> keep whatever we already have."""
        try:
            with open(self._gains_path) as f:
                g = json.load(f)
        except (OSError, ValueError):
            return

        def num(key, cur):
            v = g.get(key, cur)
            return float(v) if isinstance(v, (int, float)) else cur

        # steering gains: yaw_kp / heave_kp, falling back to a legacy single
        # "centering_kp" (from the old one-gain file) if the split keys aren't there
        legacy = num("centering_kp", None)
        base_y = legacy if legacy is not None else self.yaw_kp
        base_h = legacy if legacy is not None else self.heave_kp
        self.yaw_kp = num("yaw_kp", base_y)
        self.heave_kp = num("heave_kp", base_h)

        self.advance_surge = num("advance_surge", self.advance_surge)
        self.orbit_strafe = num("orbit_strafe", self.orbit_strafe)
        self.max_yaw_error_for_strafe = num(
            "max_yaw_error_for_strafe", self.max_yaw_error_for_strafe
        )

        # areas are given as fractions of the frame; convert to pixel^2
        enter_frac = num("orbit_enter_frac", self.orbit_enter_area / self._full)
        exit_ratio = num(
            "orbit_exit_ratio", self.orbit_exit_area / max(self.orbit_enter_area, 1e-9)
        )
        self.orbit_enter_area = enter_frac * self._full
        self.orbit_exit_area = self.orbit_enter_area * exit_ratio

        self.orbit_to_win_s = num("orbit_to_win_s", self.orbit_to_win_s)
        self.celebrate_s = num("celebrate_s", self.celebrate_s)
        self.grace_s = num("grace_s", self.grace_s)

        self.search_yaw_command = num("search_yaw_command", self.search_yaw_command)

    def _maybe_reload(self):
        """Reload the gains file if it changed on disk (cheap mtime check)."""
        try:
            mtime = os.path.getmtime(self._gains_path)
        except OSError:
            return
        if mtime != self._gains_mtime:
            self._gains_mtime = mtime
            self._load_gains()
            print(
                f"[strategy] gains reloaded (yaw_kp={self.yaw_kp:.4g}, "
                f"heave_kp={self.heave_kp:.4g})"
            )

    def update(self, box: BoundingBox, dt: float):
        self._maybe_reload()  # pick up any live edits to strategy_gains.json

        surge = strafe = heave = yaw = 0.0
        flash = False

        # --- track target validity + which side it's on (for directional search) ---
        if box.is_valid:
            self.lost_timer = 0.0
            # remember the last-seen side so we can spin that way if we lose it:
            # left of centre -> turn left (-1), right of centre -> turn right (+1)
            self.search_dir = -1.0 if box.center[0] < self.center_x else 1.0
        else:
            self.lost_timer += dt

        # lost longer than the grace period -> switch to SEARCHING (spins below)
        if self.lost_timer > self.grace_s and self.state != "SEARCHING":
            self.state = "SEARCHING"

        # ---------------- IDLE / SEARCHING (full-circle spin, last-seen direction) ----------------
        if self.state in ("IDLE", "SEARCHING"):
            yaw = self.search_dir * abs(
                self.search_yaw_command
            )  # spin toward last-seen side
            if box.is_valid:  # target spotted -> chase it
                self.state = "ADVANCING"

        # ---------------- ADVANCING ----------------
        elif self.state == "ADVANCING":
            surge = self.advance_surge
            if box.is_valid:
                err_x = self.center_x - box.center[0]
                err_y = self.center_y - box.center[1]
                yaw = -self.yaw_kp * err_x  # +yaw = turn right -> steer toward target
                heave = self.heave_kp * err_y
                if box.area > self.orbit_enter_area:  # arrived -> orbit
                    self.state = "ORBITING"
                    self.state_timer = 0.0

        # ---------------- ORBITING ----------------
        elif self.state == "ORBITING":
            if box.is_valid:
                err_x = self.center_x - box.center[0]
                err_y = self.center_y - box.center[1]
                yaw = -self.yaw_kp * err_x  # +yaw = turn right -> steer toward target
                heave = (
                    self.heave_kp * err_y
                )  # same sign as ADVANCING (archive negated this)
                # strafe sideways to circle; ease off if badly off-centre
                scale = max(0.0, 1.0 - abs(err_x) / self.max_yaw_error_for_strafe)
                strafe = self.orbit_strafe * scale
                if box.area < self.orbit_exit_area:  # target drifted away
                    self.state = "ADVANCING"
            else:
                strafe = self.orbit_strafe  # keep circling briefly

            self.state_timer += dt
            if self.state_timer > self.orbit_to_win_s:
                self.state = "CELEBRATING"
                self.state_timer = 0.0

        # ---------------- CELEBRATING ----------------
        elif self.state == "CELEBRATING":
            self.state_timer += dt
            flash = (int(self.state_timer * 4) % 2) == 0  # blink ~4 Hz
            if self.state_timer > self.celebrate_s:
                self.state = "SEARCHING"

        return surge, strafe, heave, yaw, flash
