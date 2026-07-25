"""
strategy_full.py  --  competition-backup brain.

Ported from the archived external_controller.py ("ChaseAndCircleStrategy"). Only
the state machine, the centering math, the grace period, and the search spin were
carried over; the YOLO / camera / dual-feed code isn't needed here because
run_sim.py already hands this class a clean BoundingBox. The same BoundingBox
interface means it doesn't care whether the box comes from Unity's projected
target or from YOLO on the real sub.

Interface: BoundingBox + back_visible in, four numbers + flash out. It drops
straight into run_sim.py and onto the real sub -- but note the added third arg:

    # in run_sim.py / rov_client.py:
    from strategy_full import Strategy, BoundingBox
    surge, strafe, heave, yaw, flash = strategy.update(box, dt, back_visible)

Behaviour (competition: find the other sub's TAGGED BACK and scan it):
    IDLE/SEARCHING -> ADVANCING -> ORBITING -> SCANNING, with a grace period so a
    one-frame detection dropout doesn't flip the state. We start facing AWAY, so
    with no target in view the sub SPINS -- yaws continuously (a full-circle scan)
    while bobbing in depth, until a box reappears, then locks into the chase. The
    spin turns TOWARD the side the target was last seen (exit left -> yaw left) so
    it follows the target out; before any target is seen it spins right. Once
    close it ORBITS at a held radius until the back's tags come into view (back_visible), reversing
    direction if a long circle never reveals them, then holds steady in SCANNING
    so the tags read cleanly. The actual win -- collecting enough unique tags --
    is owned by the caller (rov_client.py), so this brain never stops on its own
    or celebrates; it just keeps a clean view of the back until control is taken.

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
import math
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

        # orbit: circle to bring the target's BACK into view; flip direction if a
        # long circle never reveals it (we may have picked the long way round, or
        # the opponent is turning to keep its back hidden).
        self.orbit_dir = 1.0  # +1 / -1 strafe sense while circling
        self.orbit_no_back_timer = 0.0
        self.orbit_flip_s = 6.0  # circle this long with no back in view -> reverse
        # radius hold: strafing to circle slowly spirals the orbit outward (the
        # strafe is tangent to the circle), and with no surge term the radius just
        # grows until the box shrinks past orbit_exit and we bounce into ADVANCING.
        # A small proportional surge on box-size error holds a steady radius instead.
        self.orbit_surge_kp = 10.0  # box-size error (frac of frame) -> surge
        self.orbit_hold_frac = 0.06  # target box size (frac) while orbiting = radius

        # scan: back is in view -> hold steady so the tags read cleanly
        self.scan_lost_timer = 0.0

        self.grace_s = 0.5  # tolerate brief detection dropouts
        self.lost_timer = 0.0

        # --- search: spin + gentle depth sweep to scan for the target ---
        self.search_yaw_command = 0.1  # spin rate while searching (magnitude)
        self.search_dir = 1.0  # +1 spin right, -1 spin left; set to last-seen side
        self.search_phase = 0.0  # phase accumulator for the depth sweep
        self.search_heave_command = 0.2  # depth-sweep amplitude while searching
        self.search_heave_period_s = 4.0  # seconds per up/down depth-sweep cycle

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

        self.orbit_flip_s = num("orbit_flip_s", self.orbit_flip_s)
        self.orbit_surge_kp = num("orbit_surge_kp", self.orbit_surge_kp)
        self.orbit_hold_frac = num("orbit_hold_frac", self.orbit_hold_frac)
        self.grace_s = num("grace_s", self.grace_s)

        self.search_yaw_command = num("search_yaw_command", self.search_yaw_command)
        self.search_heave_command = num(
            "search_heave_command", self.search_heave_command
        )
        self.search_heave_period_s = num(
            "search_heave_period_s", self.search_heave_period_s
        )

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

    def update(self, box: BoundingBox, dt: float, back_visible: bool = False):
        """box: where the other sub is (from YOLO). back_visible: are we looking at
        its TAGGED BACK right now (topside passes bool(get_tag_ids()))? The win --
        collecting enough unique tags -- is owned by the caller; this brain only
        drives the sub into a clean, steady view of the back."""
        self._maybe_reload()  # pick up any live edits to strategy_gains.json

        surge = strafe = heave = yaw = 0.0
        flash = False  # celebration + hand-off are owned by the caller now

        # --- track target validity + which side it's on (for directional search) ---
        if box.is_valid:
            self.lost_timer = 0.0
            # remember the last-seen side so we can spin that way if we lose it:
            # left of centre -> turn left (-1), right of centre -> turn right (+1)
            self.search_dir = -1.0 if box.center[0] < self.center_x else 1.0
        else:
            self.lost_timer += dt

        # whole target lost longer than the grace period -> SEARCHING (spins below)
        if self.lost_timer > self.grace_s and self.state != "SEARCHING":
            self.state = "SEARCHING"

        # ---------------- IDLE / SEARCHING (spin + gentle depth sweep) ----------------
        if self.state in ("IDLE", "SEARCHING"):
            # we start facing AWAY, so sweep a full circle to find the other sub,
            # bobbing in depth too in case it starts above/below our view.
            yaw = self.search_dir * abs(self.search_yaw_command)
            self.search_phase += dt
            heave = self.search_heave_command * math.sin(
                2.0
                * math.pi
                * self.search_phase
                / max(self.search_heave_period_s, 1e-3)
            )
            if box.is_valid:  # target spotted -> chase it
                self.state = "ADVANCING"

        # ---------------- ADVANCING (close in, centred on the box) ----------------
        elif self.state == "ADVANCING":
            surge = self.advance_surge
            if box.is_valid:
                err_x = self.center_x - box.center[0]
                err_y = self.center_y - box.center[1]
                yaw = -self.yaw_kp * err_x  # +yaw = turn right -> steer toward target
                heave = self.heave_kp * err_y
                if box.area > self.orbit_enter_area:  # arrived -> orbit for the back
                    self.state = "ORBITING"
                    self.orbit_no_back_timer = 0.0

        # ---------------- ORBITING (circle until the tagged back shows) ----------------
        elif self.state == "ORBITING":
            if back_visible:  # found the back -> stop circling and scan it
                self.state = "SCANNING"
                self.scan_lost_timer = 0.0
            elif box.is_valid:
                err_x = self.center_x - box.center[0]
                err_y = self.center_y - box.center[1]
                yaw = -self.yaw_kp * err_x  # +yaw = turn right -> steer toward target
                heave = self.heave_kp * err_y  # same sign as ADVANCING
                # strafe sideways to circle; ease off if badly off-centre
                scale = max(0.0, 1.0 - abs(err_x) / self.max_yaw_error_for_strafe)
                strafe = self.orbit_dir * self.orbit_strafe * scale
                # radius hold: nudge forward/back to keep the box at the hold size,
                # so the orbit holds a steady radius instead of spiralling outward
                # (see the __init__ note). +err -> box too small (too far) -> close in.
                area_frac = box.area / self._full
                surge = max(
                    -1.0,
                    min(1.0, self.orbit_surge_kp * (self.orbit_hold_frac - area_frac)),
                )
                if box.area < self.orbit_exit_area:  # target fled -> re-chase hard
                    self.state = "ADVANCING"
            else:
                strafe = self.orbit_dir * self.orbit_strafe  # keep circling briefly

            # circled a long time and still no back -> try the other way round
            if self.state == "ORBITING":
                self.orbit_no_back_timer += dt
                if self.orbit_no_back_timer > self.orbit_flip_s:
                    self.orbit_dir *= -1.0
                    self.orbit_no_back_timer = 0.0

        # ---------------- SCANNING (back in view -> hold steady, read tags) ----------------
        elif self.state == "SCANNING":
            if back_visible:
                self.scan_lost_timer = 0.0
            else:
                self.scan_lost_timer += dt
            if box.is_valid:
                # only centre on the target; surge/strafe stay ~0 so the frame is
                # steady -- a still image reads tags far faster than strafing past.
                err_x = self.center_x - box.center[0]
                err_y = self.center_y - box.center[1]
                yaw = -self.yaw_kp * err_x
                heave = self.heave_kp * err_y
            # lost the back for longer than the grace -> circle again to re-acquire
            if self.scan_lost_timer > self.grace_s:
                self.state = "ORBITING"
                self.orbit_no_back_timer = 0.0

        return surge, strafe, heave, yaw, flash
