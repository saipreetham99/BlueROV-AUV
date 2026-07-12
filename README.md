# BlueROV2 Simulator + Real-Sub Tooling

A matched pair of systems for a custom BlueROV2 (r1) underwater vehicle: a
physics simulator built in Unity, and a real-sub control/telemetry stack that
speaks the same command language. The goal is to make the simulated sub and the
real sub behave the same way, so control strategies — hand-tuned state machines
today, reinforcement-learning policies later — can be developed against the
simulator and deployed to hardware with no rewrite.

The two halves share one command path (a six-thruster mix plus a light channel)
and one telemetry idea (depth and heading streamed back topside). Tune the
simulator once against real-world measurements and the same controller drives
both.

---

## Why this exists

Testing control code on real hardware is slow, risky, and pool-time-limited. A
simulator removes those limits — but only if it moves like the real thing. This
project treats that fidelity as the central problem:

1. Drive the real sub with fixed, repeatable commands and measure how far it
   travels (surge, strafe, heave, yaw).
2. Reproduce those same commands in the simulator and compare.
3. Adjust the simulator's hydrodynamic gains until simulated and real distances
   agree.

Once the two agree, a controller written against the simulator is trustworthy on
hardware. The vision model, state-machine strategy, and eventual RL layer all sit
on top of this matched foundation.

---

## Repository layout

The project is three cooperating pieces: code that runs on the Raspberry Pi
inside the sub, code that runs on the topside laptop, and the Unity simulator.

### Sub side — runs on the Raspberry Pi

`rov_server.py` is the single server process on the sub. It runs three
independent threads so a camera or sensor fault can never stall the
safety-critical thruster loop: a thruster receiver (UDP, applies the six-thruster
mix plus light to the PCA9685 with a watchdog), a video sender (UDP, JPEG frames
topside), and a sensor sender (UDP, depth and heading topside). `Ctrl+C`
neutralizes the thrusters and disables the PCA output cleanly.

`requirements-server.txt` lists the pip-installable dependencies and documents
the local hardware drivers that are not on PyPI.

### Topside — runs on the laptop

`rov_client.py` is the unified operator application: a single pygame window with
a thruster-test panel on the left (eight motion buttons, duration and amplitude
steppers, sensor-capture toggle, light and AprilTag-flash toggles, an emergency
STOP, and a status log) and the live video feed on the right (rendered inside
pygame, with optional AprilTag and YOLO overlays and a clean-stream recorder).
Live depth and heading are shown as they arrive, and after every thruster test
the client reports the change in depth and heading automatically.

`pool_test.py` is a command-line alternative to the client's test panel: it sends
one degree of freedom at a fixed level for a fixed duration through the exact same
mix and packet format, then returns to neutral. Useful for scripted or headless
runs.

`real_test_gui.py` is an earlier standalone button UI for the fixed-command
tests. The unified `rov_client.py` supersedes it; it is kept for reference and
simple bench use.

`requirements-client.txt` lists the topside dependencies, marking AprilTag and
YOLO as optional.

### Bench tools

`light_finder.py` identifies which PCA9685 channel the light is wired to. It
blinks one channel at a time between the light's off and on pulse while holding
the thrusters neutral, so you watch for the flash and read off the channel. It
skips the thruster channels and the camera servo channel so nothing spins or
swings while you test.

`camera_aim.py` points the camera servo (PCA9685 channel 15). You type pulse
values or nudge up and down while watching the feed, and the aim is held in the
PCA register so it persists across a server restart.

### Simulator — Unity (C#)

The Unity project models the sub's underwater physics and reproduces the real
command path. These scripts will be merged into this repository; they are listed
here so the README describes the finished system.

`Hydrodynamics.cs` implements the six-degree-of-freedom underwater dynamics:
restoring forces (weight and buoyancy, with the centre of buoyancy above the
centre of gravity for self-righting), anisotropic added mass, linear and
quadratic damping, and added-mass Coriolis coupling. `SubController.cs` reads a
gamepad and keyboard, runs commands through the thruster mix, and applies the
resulting body forces and torques. `ThrusterMixer.cs` is a byte-for-byte copy of
the real sub's mix, including actuator saturation, so the simulator saturates
exactly as the hardware does. `TuningHarness.cs` is the fidelity workbench: it
runs a fixed command for a set duration, lets the sub coast to rest, reports both
the powered distance and the total glide distance, and suggests a gain adjustment
from a real measured distance. `Pool.cs` defines the pool geometry.

---

## The command path

Both the real sub and the simulator accept the same four intents — surge,
strafe, heave, yaw — each on the range −1 to 1. These are mixed into six thruster
outputs (four horizontal, two vertical) with the same formula on both sides, and
each output is clamped, which reproduces the real actuator saturation. Clamping
matters for fidelity: at high combined commands the real thrusters saturate and
couple the axes, and the simulator must do the same.

On the sub, each thruster value becomes a PWM pulse (1500 µs neutral, scaled by an
amplitude term) written to the PCA9685. The light rides in the same packet as a
seventh value (1100 µs off, 1900 µs on). A watchdog on the thruster loop returns
everything to neutral if commands stop arriving, so a dropped connection fails
safe.

---

## Telemetry

Two quantities come back topside during a test, chosen so they can be measured
without an overhead camera (the pool has none):

Depth is read from an MS5837 pressure sensor and streamed in metres. The client
displays it in centimetres for readability while keeping metres on the wire and in
logs, so it stays unit-matched to the simulator.

Heading is integrated from the ICM20602 gyroscope's yaw rate, zeroed at startup.
This is a relative turn measurement, which is exactly what the short powered yaw
tests need, and it is immune to the magnetic interference that thruster currents
would inject into a magnetometer-based compass. (A Mahony sensor-fusion filter
using the magnetometers exists in the archive for absolute long-duration heading,
but it is deliberately not used for the powered-burst tests.)

Surge and strafe distances, which have no onboard sensor, are measured with a tape
measure along the pool deck — run long, average a few runs.

---

## Setup

### Sub (Raspberry Pi)

Install the pip dependencies, then confirm the local hardware drivers import.

```
pip install -r requirements-server.txt
python -c "import pca9685, icm20602, ms5837"
```

The three drivers (`pca9685`, `icm20602`, `ms5837`) are your own files or clones,
not PyPI packages — make sure they are importable on the Pi. On the Pi, the I2C,
SPI, and GPIO packages usually need to install against the system Python or a
virtual environment created with access to system site-packages.

A credentials file at `~/.rov_server_creds` holds the network settings, with
`[lan]` and `[wifi]` sections giving `rov_ip` and `client_ip`, and a `[DEFAULT]`
section giving the ports for sensors, thrusters, and video.

Start the server:

```
python rov-server.py            # LAN
python rov-server.py --wifi     # WiFi
```

### Topside (laptop)

```
pip install -r requirements-client.txt
python rov-client.py                     # video-only overlays
python rov-client.py --weights best.pt   # enable YOLO detection
```

A matching `~/.rov_client_creds` holds the same network settings on the topside
machine.

---

## Running a fidelity test

The loop that makes the simulator trustworthy:

1. At the pool, pick a degree of freedom and run a fixed-command test from the
   client (or `pool_test.py`). The sub runs the command for the set duration and
   then coasts to a stop.
2. Record where it comes to rest. Depth and heading are captured automatically and
   reported as a change; surge and strafe are measured with the tape.
3. Back at the desk, run the same command in the Unity `TuningHarness`, enter the
   real measured distance, and read the suggested gain.
4. Apply the gain, re-run in simulation, and confirm the simulated total distance
   matches the real one. Repeat per axis.

The harness measures total distance including the coast-to-rest glide, matching
how the real distance is measured (where the sub actually stops), so the two are
directly comparable.

---

## Notes on the light and camera

The light is controlled from the client with a simple on/off toggle, and a second
toggle flashes the light whenever an AprilTag is detected in the feed (the flash
overrides the manual state while a tag is in view). If the light does not respond,
confirm it is wired to a PCA9685 channel and use `light_finder.py` to identify
that channel, then set it in the server. The camera points via a servo on channel
15; use `camera_aim.py` to aim it.

---

## Roadmap

This README describes the tooling foundation as finished. The layers built on top
of it, in order:

1. **Simulator tuning** — match the simulated sub to the real sub per axis using
   the fidelity loop above. (Current stage.)
2. **Vision model** — an object detector on the camera feed, on both the real feed
   and a simulated camera in Unity.
3. **State-machine strategy** — a search / advance / orbit / celebrate behavior
   that turns a detection into control commands.
4. **State-machine code and RL** — the strategy implemented against the shared
   command path, and later a learned policy trained in simulation.

A planned architecture step is a port-mirror mode in which the Unity simulator
answers the same UDP protocol as the real sub — receiving the thruster packet and
streaming video and telemetry back — so the identical topside controller drives
either the simulator or the hardware with no code change.

---

## Safety

The thruster loop fails safe: if commands stop arriving, a watchdog returns the
thrusters to neutral. The client sends a steady idle keep-alive so the light and
watchdog stay live even when no test is running, and it neutralizes on quit. The
bench tools hold the thrusters neutral and leave the camera servo untouched. When
testing anywhere near the thrusters out of water, keep clear of the propellers.
