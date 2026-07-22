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
import random
import time


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


# random_number = random.randint(1,10)


class Strategy:
    """The state machine. Right now it knows two modes: SEARCH and CHASE.

    You will grow it, one small step at a time (see the TODOs lower down).
    """

    def __init__(self, camera_width: int = 640, camera_height: int = 480):
        # Middle of the image. Handy later for steering toward the target.
        self.center_x = camera_width / 2
        self.center_y = camera_height / 2

        # The sub always starts by looking around.
        self.state = "SEARCH"
        self.heave_timer = 0
        self.over_timer = 0
        self.grace_period = 0
        self.orbit_to_over_timer = 0

    def update(self, box: BoundingBox, dt: float):
        """Called ~50 times a second.
        `box` = what we see now.  `dt` = seconds since the last call (~0.02).
        Returns: surge, strafe, heave, yaw, flash_lights
        """

        # Start every step by assuming we don't move; each state fills these in.
        surge = strafe = heave = yaw = 0.0
        flash = False

        # -------- STATE: SEARCH ----------------
        if self.state == "SEARCH":
            # We can't see the target. Spin slowly in place to look for it.
            yaw = -0.3
            # heave = 0.5

            self.heave_timer += dt
            if self.heave_timer > 2:
                heave = -0.1
            if self.heave_timer > 4:
                heave = 0.2
                if self.heave_timer > 6:
                    heave = 0
                    self.heave_timer = 0
            # if self.center_y

            # Transition: the moment we see it, start chasing.
            if box.is_valid and box.area < 11000:
                print(" CHASE (target found)")
                self.state = "CHASE"

        # ---------------- STATE: CHASE ----------------
        if self.state == "CHASE":
            # Transition: if we lose sight of it, go back to searching.
            surge = 0.7
            if not box.is_valid:
                self.state = "SEARCH"
                print("CHASE -> SEARCH (target lost)")
            elif box.area > 11000:  # and box.center[0] < self.center_x:
                self.state = "ORBIT"
                # elif box.area > 11000 and box.center[0] > self.center_x:
                # self.state = "OVER"
            else:
                # We can see the target. Drive forward toward it.
                error_x = self.center_x - box.center[0]
                yaw = -0.0025 * error_x

                error_y = self.center_y - box.center[1]
                heave = 0.0055 * error_y

        if self.state == "ORBIT":
            if not box.is_valid:
                print("ORBIT -> SEARCH (target LOST)")
                self.grace_period += dt
                if self.grace_period > 0.5:
                    self.state = "SEARCH"
                    self.grace_period = 0

            else:
                self.orbit_to_over_timer += dt
                if self.orbit_to_over_timer < 8:
                    error_x = self.center_x - box.center[0]
                    error_y = self.center_y - box.center[1]
                    yaw = -0.3
                    strafe = 0.4
                    surge = 0.5
                elif self.orbit_to_over_timer > 8:
                    self.state = "OVER"
                    self.orbit_to_over_timer = 0
        # if self.state == "CELEBRATE!":

        if self.state == "OVER":
            self.over_timer += dt
            if self.over_timer > 0.5:
                heave = 0.15
            if self.over_timer > 1:
                surge = 0.65
            if self.over_timer > 2.5:
                yaw = -0.6
            if self.over_timer > 3.5:
                heave = -0.2
            if self.over_timer > 5.5:
                self.state = "SEARCH"
                self.over_timer = 0

                # pitch = 0.1

            # ===========================================================
            # TODO  LEVEL 3 -- CENTERING (keep the target in the middle)
            #   The target might be off to one side. Steer so it stays centered.
            #   Idea:
            #       error_x = self.center_x - box.center[0]
            #       yaw = 0.0005 * error_x        # turn toward it
            #   Try it. Which way does it turn if the target is on the right?
            #   Do the same with error_y and `heave` to keep it vertically centered.
            # ===========================================================

            # ===========================================================
            # TODO  LEVEL 4 -- RANGE (know when we've arrived)
            #   As we get closer, box.area gets bigger. Pick a target area, e.g.
            #       if box.area > 15000:
            #           self.state = "ARRIVED"     # (a new state you add)
            #   In ARRIVED you might stop, or move on to orbiting.
            # ===========================================================

        # ===========================================================
        # TODO  LEVEL 5 -- ORBIT (circle around the target)
        #   Add an "ORBIT" state. In it: keep centering (from Level 3) AND
        #   strafe sideways so you circle the target instead of ramming it.
        #       strafe = 0.6
        # ===========================================================

        # ===========================================================
        # TODO  LEVEL 6 -- ROBUSTNESS
        #   * Grace period: don't switch to SEARCH the instant the box blinks
        #     out for one frame. Count how long it's been missing (using dt)
        #     and only give up after, say, 0.5 seconds.
        #   * Win timer: after orbiting for N seconds, switch to a "CELEBRATE"
        #     state and set flash = True to blink the lights.
        # ===========================================================

        return (
            surge,
            strafe,
            heave,
            yaw,
            flash,
        )
