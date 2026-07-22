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
    """The state machine. Right now it knows two modes: SEARCH and CHASE.

    You will grow it, one small step at a time (see the TODOs lower down).
    """

    def __init__(self, camera_width: int = 640, camera_height: int = 480):
        # Middle of the image. Handy later for steering toward the target.
        self.center_x = camera_width / 2
        self.center_y = camera_height / 2

        # The sub always starts by looking around.
        self.state = "SEARCH"
        self.time_in_orbit = 0
        self.target_in_frame = 0
        self.burst_timer_threshold = 0.5
        self.b_timer = 0

    def update(self, box: BoundingBox, dt: float):
        """Called ~50 times a second.
        `box` = what we see now.  `dt` = seconds since the last call (~0.02).
        Returns: surge, strafe, heave, yaw, flash_lights
        """

        # Start every step by assuming we don't move; each state fills these in.
        surge = strafe = heave = yaw = 0.0
        flash = False

        # ---------------- STATE: SEARCH ----------------
        if self.state == "SEARCH":
            # We can't see the target. Spin slowly in place to look for it.
            yaw = 0.3

            # Transition: the moment we see it, start chasing.
            if box.is_valid:
                self.state = "CHASE"
                print("SEARCH -> CHASE (target found)")

        # ---------------- STATE: CHASE ----------------
        elif self.state == "CHASE":
            # We can see the target. Drive forward toward it.
            surge = 0.7

            # Transition: if we lose sight of it, go back to searching.
            if not box.is_valid:
                self.state = "SEARCH"
                print("CHASE -> SEARCH (target lost)")
            else:
                error_x = self.center_x - box.center[0]
                yaw = -0.0030 * error_x
                error_y = self.center_y - box.center[1]
                heave = 0.0015 * error_y

            if box.area > 15000:
                self.state = "ORBIT"
                print("CHASE -> ORBIT (target in range)")

        # ----------------- STATE ORBIT:  ------------------
        elif self.state == "ORBIT":
            # surge = 0
            # ime_in_orbit = 3

            if box.area < 11000:
                self.state = "CHASE"
                print("ORBIT -> CHASE (target out of range)")
            else:
                error_x = self.center_x - box.center[0]
                yaw = -0.0070 * error_x
                error_y = self.center_y - box.center[1]
                heave = 0.0015 * error_y
                strafe = 0.4
                surge = 0.5

                self.target_in_frame += dt
                if not box.is_valid and self.target_in_frame > 0.5:
                    self.state = "SEARCH"
                    print("ORBIT -> SEARCH (target lost)")

                self.time_in_orbit += dt
                if self.time_in_orbit > 6:
                    self.state = "BURST"
                    print("ORBIT -> BURST (escaping orbital lock)")
                # if(self.time_in_orbit>6 ):

        elif self.state == "BURST":
            self.b_timer += dt
            if self.b_timer <= self.burst_timer_threshold:
                heave = 0.8
                print("Heaving")
            elif self.b_timer <= self.burst_timer_threshold + 3:
                surge = 0.8
                yaw = 0.7
                print("Surging and Yawing")
        #    elif(self.b_timer <= self.burst_timer_threshold + 1 ):
        #         yaw = 0. 8
        #         print("Ywaing")

        # ---------------- STATE: CELEBRATE ------------------

        elif self.state == "CELEBRATING":
            error_x = self.center_x - box.center[0]
            yaw = -0.0003 * error_x
            error_y = self.center_y - box.center[1]
            heave = 0.0002 * error_y
            flash = True

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

        return surge, strafe, heave, yaw, flash
