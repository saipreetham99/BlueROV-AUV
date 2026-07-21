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

Anti-stall: two subs each centre on the other, so each keeps its back pointed
    away -- a plain orbit counter-rotates and never gets behind an active tracker,
    and any shared downward bias slowly sinks BOTH to the floor. Three additions
    push back on that:
      * DASH -- right after each direction flip, whip around at full strafe with
        NO centring-ease for a moment, trying to swing behind an opponent that
        keeps re-facing us. It's still an orbit (yaw stays centred, back hidden),
        just a faster one.
      * RESET maneuver -- if BOTH orbit directions fail to reveal the back we're
        likely stalemated. Rising alone isn't enough (both subs rise together and
        end stacked vertically, out of view, spinning forever), so we RISE, drive
        FORWARD, then SPIN AROUND quickly -- changing depth, position AND heading
        so the other sub lands back in view -- then hand to SEARCHING. The
        vertical phase ALTERNATES each time this fires (up on the first lock,
        then down, then up, ...) so repeated resets can't just pin us against
        the surface or the floor.
      * SEARCH timeout -- if we've already met the opponent once and searching
        drags on (stuck stacked/spinning), run the RESET maneuver again to
        reposition, so we can never get pinned in an endless spin.

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
        # Aim at the target's CAMERA point (TOP of the box), not its body centre.
        # x stays the horizontal centre; y is the body centre (y + height/2)
        # shifted UP by height/2 -> y, i.e. the top edge. Holding the box TOP at
        # image centre keeps the target low in the frame, so the sub biases
        # UPWARD -- when both are on the floor they each drive up and lift off.
        return (self.x + self.width / 2, self.y)

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
        # ADVANCING closes in at an ANGLE, not head-on: hold the target this many
        # pixels off-centre (positive = right of centre -> approach from its left).
        self.approach_offset_px = 80.0
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

        # --- symmetry-breaker: a short DASH right after each direction flip ---
        # A plain orbit stays face-on (both subs counter-rotate to keep facing each
        # other), so it never gets behind an active tracker. Right after a flip we
        # whip around at FULL strafe with no centring-ease for a moment, to try to
        # out-circle them. yaw-centring stays on, so the back never comes around.
        self.dash_s = 1.5  # how long each post-flip dash lasts (s)
        self.dash_strafe = 1.0  # strafe speed during a dash (max = fastest circle)
        self.dash_timer = 0.0  # >0 while a dash is in progress

        # --- RESET maneuver: break a stuck face-off, then re-acquire ---
        # If BOTH orbit directions fail to reveal the back, we're stalemated and
        # slowly sinking. Rising ALONE isn't enough: both subs rise together and
        # end stacked vertically, out of each other's view, then spin forever
        # without re-finding each other. So reposition deliberately -- RISE, drive
        # FORWARD, then SPIN AROUND quickly -- changing depth, position AND heading
        # so the other sub lands back in view. Rising also lifts us off the floor.
        self.stuck_reset_s = 14.0  # total stuck-orbit time (spans both flips) -> reset
        self.reset_heave = 0.5  # upward heave, phase 1
        self.reset_up_s = 1.0  # phase 1 duration: rise (s)
        self.reset_fwd_surge = 0.6  # forward surge, phase 2
        self.reset_fwd_s = 1.0  # phase 2 duration: drive forward (s)
        self.reset_turn_yaw = 0.8  # yaw rate, phase 3 (strong = quick turn-around)
        self.reset_turn_s = 1.0  # phase 3 duration: spin around (s)
        # vertical phase ALTERNATES each reset so repeated resets can't pin us at
        # the surface (or floor): up on the 1st lock, then down, then up, ...
        self.reset_heave_dir = 1.0  # +1 = up, -1 = down (starts up)
        self.stuck_timer = 0.0  # stuck-orbit clock; resets on back-found / reset
        self.reset_timer = 0.0  # ELAPSED time inside the reset maneuver

        # --- search timeout: don't spin forever after we've already met the sub ---
        # If searching drags on AFTER first contact, we're probably stuck stacked /
        # spinning -> run the RESET maneuver again to reposition. Gated by
        # seen_target_once so the initial hunt at match start is left alone.
        self.search_timeout_s = 8.0  # search this long (post-contact) -> reposition
        self.search_timer = 0.0
        self.seen_target_once = False

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
        self.approach_offset_px = num("approach_offset_px", self.approach_offset_px)
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

        # symmetry-breaker (dash)
        self.dash_s = num("dash_s", self.dash_s)
        self.dash_strafe = num("dash_strafe", self.dash_strafe)

        # RESET maneuver + search-timeout knobs
        self.stuck_reset_s = num("stuck_reset_s", self.stuck_reset_s)
        self.reset_heave = num("reset_heave", self.reset_heave)
        self.reset_up_s = num("reset_up_s", self.reset_up_s)
        self.reset_fwd_surge = num("reset_fwd_surge", self.reset_fwd_surge)
        self.reset_fwd_s = num("reset_fwd_s", self.reset_fwd_s)
        self.reset_turn_yaw = num("reset_turn_yaw", self.reset_turn_yaw)
        self.reset_turn_s = num("reset_turn_s", self.reset_turn_s)
        self.search_timeout_s = num("search_timeout_s", self.search_timeout_s)

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
            self.seen_target_once = True  # we've met the opponent at least once
            # remember the last-seen side so we can spin that way if we lose it:
            # left of centre -> turn left (-1), right of centre -> turn right (+1)
            self.search_dir = -1.0 if box.center[0] < self.center_x else 1.0
        else:
            self.lost_timer += dt

        # whole target lost longer than the grace period -> SEARCHING (spins below).
        # BUT never bail to SEARCHING while running the RESET maneuver or mid-dash:
        # both keep us moving deliberately, and an abort would drop us into the very
        # spin-in-place we're trying to escape.
        dashing = self.state == "ORBITING" and self.dash_timer > 0.0
        if (
            self.lost_timer > self.grace_s
            and self.state not in ("SEARCHING", "RESET")
            and not dashing
        ):
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
                self.search_timer = 0.0
            else:
                # can't see it. If we've met it before and we've been hunting too
                # long, we're probably stuck stacked/spinning -> reposition again.
                self.search_timer += dt
                if self.seen_target_once and self.search_timer > self.search_timeout_s:
                    self.state = "RESET"
                    self.reset_timer = 0.0
                    self.search_timer = 0.0

        # ---------------- ADVANCING (close in, centred on the box) ----------------
        elif self.state == "ADVANCING":
            surge = self.advance_surge
            if box.is_valid:
                # aim OFF-centre horizontally so we close in at an angle rather
                # than nose-to-nose (head-on just parks us facing an opponent who
                # is facing us). Vertical stays centred.
                err_x = (self.center_x + self.approach_offset_px) - box.center[0]
                err_y = self.center_y - box.center[1]
                yaw = -self.yaw_kp * err_x  # +yaw = turn right -> steer toward target
                heave = self.heave_kp * err_y
                if box.area > self.orbit_enter_area:  # arrived -> orbit for the back
                    self.state = "ORBITING"
                    self.orbit_no_back_timer = 0.0
                    self.stuck_timer = 0.0
                    self.dash_timer = 0.0

        # ---------------- ORBITING (circle until the tagged back shows) ----------------
        elif self.state == "ORBITING":
            if back_visible:  # found the back -> stop circling and scan it
                self.state = "SCANNING"
                self.scan_lost_timer = 0.0
                self.stuck_timer = 0.0
            else:
                # how long we've been stuck circling with no back in view. This
                # clock spans BOTH direction flips -- it only resets when we
                # actually find the back, or after a reset maneuver.
                self.stuck_timer += dt

                if self.stuck_timer > self.stuck_reset_s:
                    # both directions tried and still no back -> run reset maneuver
                    self.state = "RESET"
                    self.reset_timer = 0.0
                    # start the vertical phase this tick (no dead frame); its
                    # direction alternates per reset via reset_heave_dir
                    heave = self.reset_heave_dir * self.reset_heave
                elif box.is_valid:
                    err_x = self.center_x - box.center[0]
                    err_y = self.center_y - box.center[1]
                    yaw = (
                        -self.yaw_kp * err_x
                    )  # keep facing target -> back stays hidden
                    heave = self.heave_kp * err_y  # same sign as ADVANCING

                    # strafe sideways to circle. NORMALLY ease off when badly
                    # off-centre; during a DASH hold full strafe and skip the ease,
                    # to whip around faster than the opponent can re-face us.
                    if self.dash_timer > 0.0:
                        self.dash_timer -= dt
                        strafe = self.orbit_dir * self.dash_strafe
                    else:
                        scale = max(
                            0.0, 1.0 - abs(err_x) / self.max_yaw_error_for_strafe
                        )
                        strafe = self.orbit_dir * self.orbit_strafe * scale

                    # radius hold: nudge fwd/back to keep the box at the hold size,
                    # so the orbit holds a steady radius instead of spiralling out.
                    # +err -> box too small (too far) -> close in.
                    area_frac = box.area / self._full
                    surge = max(
                        -1.0,
                        min(
                            1.0,
                            self.orbit_surge_kp * (self.orbit_hold_frac - area_frac),
                        ),
                    )
                    if box.area < self.orbit_exit_area:  # target fled -> re-chase hard
                        self.state = "ADVANCING"
                        self.stuck_timer = 0.0
                        self.dash_timer = 0.0
                else:
                    # box briefly gone: keep circling (dash speed if mid-dash)
                    if self.dash_timer > 0.0:
                        self.dash_timer -= dt
                        strafe = self.orbit_dir * self.dash_strafe
                    else:
                        strafe = self.orbit_dir * self.orbit_strafe

            # circled a long time in THIS direction and still no back -> flip, and
            # kick off a short dash in the new direction to try to get behind them.
            if self.state == "ORBITING":
                self.orbit_no_back_timer += dt
                if self.orbit_no_back_timer > self.orbit_flip_s:
                    self.orbit_dir *= -1.0
                    self.orbit_no_back_timer = 0.0
                    self.dash_timer = self.dash_s  # dash in the new direction

        # ---------------- RESET (break a stuck face-off, then re-acquire) --------------
        # Rising alone leaves both subs stacked vertically, spinning forever without
        # re-finding each other. So reposition deliberately: RISE, drive FORWARD, then
        # SPIN AROUND quickly -- changing depth, position AND heading -- to bring the
        # other sub back into view. (The spin does briefly swing our back around; that
        # is the accepted cost of getting unstuck.) Then hand to SEARCHING to lock on.
        elif self.state == "RESET":
            if back_visible:  # stumbled onto the back mid-maneuver -> scan it
                self.state = "SCANNING"
                self.scan_lost_timer = 0.0
                self.stuck_timer = 0.0
            else:
                self.reset_timer += dt
                t = self.reset_timer
                if t < self.reset_up_s:  # phase 1: rise or dive (alternates per reset)
                    heave = self.reset_heave_dir * self.reset_heave
                elif t < self.reset_up_s + self.reset_fwd_s:  # phase 2: drive forward
                    surge = self.reset_fwd_surge
                elif t < self.reset_up_s + self.reset_fwd_s + self.reset_turn_s:
                    # spin the SAME way the follow-on SEARCH will (search_dir), so
                    # the reset turn and the search spin are one continuous turn --
                    # no reversal that would sweep our back past the opponent twice.
                    yaw = self.search_dir * abs(self.reset_turn_yaw)  # phase 3
                else:  # done -> flip vertical dir for next time, go find the target
                    self.reset_heave_dir *= -1.0  # next reset goes the other way
                    self.state = "SEARCHING"
                    self.reset_timer = 0.0
                    self.search_timer = 0.0

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
                self.stuck_timer = 0.0
                self.dash_timer = 0.0

        return surge, strafe, heave, yaw, flash
