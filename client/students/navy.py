"""
strategy.py  --  THE SUB'S BRAIN.  This is the ONLY file you edit.

Each step, the brain is given ONE thing: a bounding box that says where the
target is on the camera (and how big it looks). It must return HOW TO MOVE:

    surge   +forward / -backward
    strafe  +right   / -left
    heave   +up      / -down
    yaw     +turn right / -turn left

Every number is between -1 and 1 (0 = don't move that way).

>>> THE GOLDEN RULE <<<
This file must NEVER mention cameras, networks, Unity, or the Pi. It only ever
deals with a bounding box coming in and four numbers going out. Because of that,
the EXACT same file runs in the simulator and on the real sub. If you put camera
or network code in here, you break that, and your work stops transferring.
"""

from dataclasses import dataclass
from typing import Tuple


# ---- tuning knobs (all the numbers live here) ---------------------------------
# ORBIT has HYSTERESIS: enter when the box grows past ENTER, only fall back to
# CHASE when it shrinks below EXIT. EXIT must stay BELOW ENTER (it already did
# here), otherwise the state flip-flops between CHASE and ORBIT every tick.
ORBIT_ENTER_AREA = 1200.0  # box this big -> close enough to orbit
ORBIT_EXIT_AREA = 900.0  # box shrank this small -> target fled, chase again
ORBIT_HOLD_AREA = 1500.0  # box size we try to HOLD while orbiting (= radius)
ORBIT_SURGE_KP = 0.001  # box-area error -> surge, to hold that radius
ORBIT_TIME_S = 12.0  # orbit this long -> BURST out of the orbital lock

BURST_TIME_S = 12.0  # total BURST duration, then re-enter ORBIT
BURST_BACKUP_S = 1.0  # phase 1: back off, then circle the other way

GRACE_S = 0.5  # tolerate a detection dropout this long before SEARCH


def clamp(v: float) -> float:
    """Thruster commands must stay inside -1 .. 1."""
    return max(-1.0, min(1.0, v))


@dataclass
class BoundingBox:
    """Where the target is on the camera image (640 x 480 pixels).

    (x, y) is the TOP-LEFT corner of the box; width/height are its size.
    If the sub can't see the target, the box is empty (width and height 0).
    """

    x: float = 0.0
    y: float = 0.0
    width: float = 0.0
    height: float = 0.0

    @property
    def center(self) -> Tuple[float, float]:
        """Middle of the box: (center_x, center_y)."""
        return (self.x + self.width / 2, self.y + self.height / 2)

    @property
    def area(self) -> float:
        """How big the box looks. Bigger area = target is closer."""
        return self.width * self.height

    @property
    def is_valid(self) -> bool:
        """True only when we can actually see the target."""
        return self.width > 0 and self.height > 0


class Strategy:
    """The state machine: SEARCH -> CHASE -> ORBIT -> BURST -> ORBIT."""

    def __init__(self, camera_width: int = 640, camera_height: int = 480):
        # Middle of the image. Handy later for steering toward the target.
        self.center_x = camera_width / 2
        self.center_y = camera_height / 2

        # The sub always starts by looking around.
        self.state = "SEARCH"
        self.time_searching = 0.0

        # ORBIT clock: reset EVERY time we enter ORBIT, so time spent in earlier
        # orbits can't add up and fire BURST the instant we arrive.
        self.time_in_orbit = 0.0

        # BURST clock: reset when we ENTER burst and when we LEAVE it. (If it is
        # only reset on the way out, the second BURST starts already expired and
        # bounces straight back to ORBIT.)
        self.b_timer = 0.0

        self.yaw_direction = -1

        # --- grace period ---------------------------------------------------
        # When the box blinks out we don't go blind: for GRACE_S seconds we keep
        # steering on the LAST box we actually saw.
        self.grace_s = GRACE_S
        self.lost_timer = 0.0
        self._last_box = BoundingBox()

    def update(self, box: BoundingBox, dt: float):
        """Called ~50 times a second.
        `box` = what we see now.  `dt` = seconds since the last call (~0.02).
        Returns: surge, strafe, heave, yaw, flash_lights
        """

        # Start every step by assuming we don't move; each state fills these in.
        surge = strafe = heave = yaw = 0.0
        flash = False

        # -------- GRACE PERIOD BOOKKEEPING ----------------------------------
        if box.is_valid:
            self.lost_timer = 0.0
            self._last_box = box  # remember it, to coast on during a dropout
        else:
            self.lost_timer += dt

        # box gone, but only BRIEFLY -> coast on the last known box.
        coasting = (
            not box.is_valid
            and self.lost_timer <= self.grace_s
            and self._last_box.is_valid
        )
        # `aim` is the box the STEERING math uses. When we can see the target it
        # IS the live box, so this is a no-op outside the dropout window.
        aim = self._last_box if coasting else box

        # ---------------- STATE: SEARCH ----------------
        if self.state == "SEARCH":
            # We can't see the target. Spin slowly in place to look for it.
            # Transition: the moment we see it, start chasing.
            yaw = 0.3
            self.time_searching += dt
            if box.is_valid:  # live box only -- never re-acquire on a stale one
                self.state = "CHASE"
                print("SEARCH -> CHASE (target found)")
            # if self.time_searching > 5 and self.time_searching < 7:
            #     heave = -0.3

        # ---------------- STATE: CHASE ----------------
        elif self.state == "CHASE":
            # We can see the target. Drive forward toward it.
            surge = 0.7

            # Transition: only give up once the box has been missing for longer
            # than the grace period.
            if not box.is_valid and not coasting:
                self.state = "SEARCH"
                print("CHASE -> SEARCH (target lost)")
            elif aim.is_valid:
                # live box, or the last known box while coasting
                error_x = self.center_x - aim.center[0]
                yaw = -0.002 * error_x
                error_y = self.center_y - aim.center[1]
                heave = 0.0015 * error_y

                # Arrived. Reset the orbit clock so BURST fires ORBIT_TIME_S
                # from NOW, not from however long past orbits added up to.
                if aim.area > ORBIT_ENTER_AREA:
                    self.state = "ORBIT"
                    self.time_in_orbit = 0.0
                    print("CHASE -> ORBIT (target in range)")

        # ----------------- STATE ORBIT:  ------------------
        elif self.state == "ORBIT":
            # Lost for real (past the grace window) -> go look for it. Checked
            # BEFORE the area test, because a missing box reports area 0 and
            # would otherwise read as "target fled" on a single dropped frame.
            if not box.is_valid and not coasting:
                self.state = "SEARCH"
                print("ORBIT -> SEARCH (target lost)")
            elif aim.area < ORBIT_EXIT_AREA:
                self.state = "CHASE"
                print("ORBIT -> CHASE (target out of range)")
            else:
                # live box, or the last known box while coasting
                error_x = self.center_x - aim.center[0]
                yaw = self.yaw_direction * 0.0070 * error_x
                error_y = self.center_y - aim.center[1]
                heave = 0.0015 * error_y

                # Circle sideways...
                strafe = 0.4
                # ...and nudge fwd/back to HOLD the radius. Strafing alone is
                # tangent to the circle, so a fixed surge spirals in (or out);
                # a small proportional surge on box size keeps it steady.
                # box too small (too far) -> positive surge -> close in.
                surge = clamp(ORBIT_SURGE_KP * (ORBIT_HOLD_AREA - aim.area))

                self.time_in_orbit += dt
                if self.time_in_orbit > ORBIT_TIME_S:
                    self.state = "BURST"
                    self.b_timer = 0.0  # start the burst clock fresh
                    print("ORBIT -> BURST (escaping orbital lock)")

        # --------------- STATE: BURST----------------------------
        # Back off for a moment, then circle the OTHER way for a while. This is a
        # timed maneuver, so it rides out a dropout instead of bailing to SEARCH.
        elif self.state == "BURST":
            self.b_timer += dt
            if self.b_timer > BURST_TIME_S:
                self.state = "ORBIT"
                self.b_timer = 0.0  # reset on the way out too
                self.time_in_orbit = 0.0
                print("BURST -> ORBIT (reenter orbit)")
            elif self.b_timer < BURST_BACKUP_S:
                surge = -0.3
            else:
                strafe = -0.4
                surge = 0.3
                # Steer on the live box, or the last known one while coasting.
                # With neither, don't steer at all: an empty box reports a centre
                # of (0, 0), which would command a hard turn at nothing.
                if aim.is_valid:
                    error_x = self.center_x - aim.center[0]
                    yaw = self.yaw_direction * 0.0070 * error_x
                    error_y = self.center_y - aim.center[1]
                    heave = 0.0015 * error_y

        # ---------------- STATE: CELEBRATE ------------------
        # (still unreachable -- left alone on purpose)
        elif self.state == "CELEBRATING":
            error_x = self.center_x - box.center[0]
            yaw = -0.0003 * error_x
            error_y = self.center_y - box.center[1]
            heave = 0.0002 * error_y
            flash = True

        # Never hand the thrusters a number outside -1 .. 1.
        return clamp(surge), clamp(strafe), clamp(heave), clamp(yaw), flash
