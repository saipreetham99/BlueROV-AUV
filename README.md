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

The stack now spans the full pipeline: the fidelity-tuning foundation, a YOLO
vision model with training tooling, a search/advance/orbit/scan state-machine
brain that runs unchanged in the sim and on the real sub, and a Unity bridge that
answers the real sub's UDP protocol so the identical controller drives either
one. The current application is a 1v1 task — find the other sub's tagged back and
hold a clean view of it while the operator side collects its tags.

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

The project is a few cooperating pieces: code that runs on the Raspberry Pi
inside the sub, code that runs on the topside laptop, the shared control brain,
the vision/dataset tooling, and the Unity simulator with its Python runners.

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

`rov_client.py` is the unified operator application: a single pygame window that
has grown well past a test panel. It now covers four modes through one shared
mix-and-packet path:

- **Thruster-test panel** — eight motion buttons, a duration stepper and a
  click-to-type amplitude box, a sensor-capture toggle, light and AprilTag-flash
  toggles, an emergency STOP, and a status log. After each test the client
  reports the change in depth and heading, split into powered and glide phases.
- **Manual gamepad control** (JOYSTICK button) — left stick surge, right stick
  strafe, right trigger heave up / right-stick-X-left heave down, XBOX/BACK yaw,
  D-pad amplitude ±100, X light, B tag-flash. Live whenever autonomy is off.
- **Autonomous chase & orbit** (AUTONOMOUS button) — a YOLO detection feeds the
  strategy brain, which drives the thrusters. Enabled only when a detector is
  loaded (`--weights`) and a strategy module imports. Detection runs in a
  separate process (torch never shares the pygame/cv2 process), and the control
  loop is event-driven — it acts on each fresh detection as it lands rather than
  polling on a timer.
- **AprilTag mission** — a "Tags to finish" target; while autonomous, each unique
  AprilTag ID seen is remembered, and reaching the target flashes the lights and
  hands control back to the gamepad. RESET TAGS clears the count.

The right side shows the live video (rendered inside pygame, "NO VIDEO" when the
stream is down) with optional AprilTag and YOLO overlays, a clean-stream mp4
recorder, and a small latency readout (see Autonomy below). A docked **tune
panel** (TUNE button) exposes every strategy gain as a slider and writes
`strategy_gains.json`, which the brain hot-reloads live.

`pool_test.py` is a command-line alternative to the client's test panel: it sends
one degree of freedom at a fixed level for a fixed duration through the exact same
mix and packet format, then returns to neutral. Useful for scripted or headless
runs.

`real_test_gui.py` is an earlier standalone button UI for the fixed-command
tests. The unified `rov_client.py` supersedes it; it is kept for reference and
simple bench use.

`requirements-client.txt` lists the topside dependencies, marking AprilTag and
YOLO as optional.

### The strategy brain

The brain is a `Strategy` class with one entry point,
`update(box, dt, back_visible)`, that takes a `BoundingBox` and returns four
movement intents plus a flash flag. Because it depends only on that interface, the
same class runs against Unity's projected target box in the sim and against YOLO's
box on the real sub with no change.

Its state machine is IDLE/SEARCHING → ADVANCING → ORBITING → SCANNING. It starts
facing away and spins (with a depth sweep) to find the other sub, closes in at an
angle, orbits at a held radius until the tagged back comes into view
(`back_visible`), then holds steady to let the tags read. A grace period tolerates
brief detection dropouts — during that window it keeps steering on the last known
box instead of going blind — before falling back to searching. Anti-stall
behavior (a post-flip DASH, a reposition RESET maneuver, and a search timeout)
keeps two mutually-tracking subs from deadlocking.

Every knob lives in `strategy_gains.json` next to the brain — the single source of
truth. It is loaded at startup and hot-reloaded on change, so edits (by hand or
via the client's tune panel) land on the next control loop, and the values tuned
in the sim are exactly what the real sub runs. The real sub picks its brain module
with `--strategy` (e.g. `strategy_full`); the sim's `run_sim.py` imports its brain
as `strategy`, and `run_evade.py` runs a frozen `strategy_target.py` copy as a
fixed sparring opponent.

### Vision model + dataset tooling

`train.py` fine-tunes YOLO26n on a Roboflow-exported dataset, auto-picking the
fastest device (CUDA → Apple MPS → CPU). Point `--data` at the export's
`data.yaml`; the trained weights land at `runs/detect/<name>/weights/best.pt`,
which is exactly what the client's `--weights` wants.

`render.py` plays a video file or webcam through a set of weights with an FPS and
inference-latency overlay — a quick way to sanity-check a model and its live
speed off-sub.

`extract_frames.py` samples frames from recorded videos at a target rate to build
a labeling set (`python extract_frames.py clip1.mp4 clip2.mp4 [fps]`), pairing
naturally with the client's mp4 recorder.

### Bench tools

`light_finder.py` identifies which PCA9685 channel the light is wired to. It
blinks one channel at a time between the light's off and on pulse while holding
the thrusters neutral, so you watch for the flash and read off the channel. It
skips the thruster channels and the camera servo channel so nothing spins or
swings while you test.

`camera_aim.py` points the camera servo (PCA9685 channel 15). You type pulse
values or nudge up and down while watching the feed, and the aim is held in the
PCA register so it persists across a server restart.

`gamepad_test.py` prints the name of every button and axis as you press them —
used to derive the controller mapping the client's manual mode relies on.

### Simulator — Unity (C#)

The Unity project models the sub's underwater physics, reproduces the real command
path, and now speaks the real sub's network protocol directly.

`Hydrodynamics.cs` implements the six-degree-of-freedom underwater dynamics
(Fossen model): restoring forces (weight and buoyancy, with the centre of buoyancy
above the centre of gravity for self-righting), anisotropic added mass, linear and
quadratic damping, and added-mass Coriolis coupling. `SubController.cs` reads a
gamepad and keyboard, runs commands through the thruster mix, and applies the
resulting body forces and torques. `ThrusterMixer.cs` is a byte-for-byte copy of
the real sub's mix, including actuator saturation, so the simulator saturates
exactly as the hardware does. `TuningHarness.cs` is the fidelity workbench: it runs
a fixed command for a set duration, lets the sub coast to rest, reports both the
powered distance and the total glide distance, and suggests a gain adjustment from
a real measured distance. `Pool.cs` defines the pool geometry.

`SimBridge.cs` is the port-mirror layer: put it on the main sub and it makes Unity
answer the same UDP protocol as the real vehicle. Each frame it projects the target
sub into the main sub's camera and sends its bounding box as four floats
(center_x, center_y, width, height in a 640×480 image; all zeros = not visible),
and it receives the real sub's 7-channel thruster packet, inverts the mix to
recover surge/strafe/heave/yaw, and drives the sub through the same force path as
manual control. `RandomSpawn.cs` teleports a sub to a random point and heading
inside the pool on Play; put it on both subs for varied match starts.

### Simulator runners — Python

`run_sim.py` connects a `Strategy` to the simulator: it receives the target's
bounding box over UDP, calls `strategy.update(box, dt)`, mixes the result into the
same `<7H` thruster packet the real sub uses, and sends it to the sim. `--mock`
runs an open-loop scripted target with no Unity, for checking state transitions.
It uses ports 60010 (Unity → Python boxes) and 60011 (Python → Unity thrusters).
Do not edit it.

`run_evade.py` runs the target sub's own frozen brain (`strategy_target.py`) by
rebinding `run_sim`'s `Strategy` and using the target's ports (60012 / 60013). Run
it alongside `run_sim.py` for a two-sub match against a fixed baseline opponent.

---

## The command path

Both the real sub and the simulator accept the same four intents — surge, strafe,
heave, yaw — each on the range −1 to 1. These are mixed into six thruster outputs
(four horizontal, two vertical) with the same formula everywhere it appears
(`rov_client.py`, `pool_test.py`, `run_sim.py`, and `ThrusterMixer.cs`), and each
output is clamped, which reproduces the real actuator saturation. Clamping matters
for fidelity: at high combined commands the real thrusters saturate and couple the
axes, and the simulator must do the same.

On the sub, each thruster value becomes a PWM pulse (1500 µs neutral, scaled by an
amplitude term) written to the PCA9685. The light rides in the same packet as a
seventh value (1100 µs off, 1900 µs on). A watchdog on the thruster loop returns
everything to neutral if commands stop arriving, so a dropped connection fails
safe. The Unity bridge honours the same watchdog: no packet for the timeout window
and the sim sub stops.

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

## Autonomy and the competition

With a detector loaded and a strategy module importable, the AUTONOMOUS toggle
closes the loop on the real sub: YOLO produces a bounding box, the strategy brain
turns it into surge/strafe/heave/yaw, and the client sends the same thruster packet
as everything else. Detection runs in its own process so torch can't collide with
pygame/cv2, and the control loop is event-driven — it runs one control step the
moment a fresh box arrives rather than re-sampling a stale one on a fixed timer.

The video panel shows a small live latency readout. `det` is the rate fresh
detections arrive (it should track the video's frame rate), and the lag figure is
how stale the current box is by the time it's consumed — shown as `ctrl lag` (the
control loop's own reading) while autonomous, or a display proxy otherwise. It's a
cheap health check: if `det` collapses or lag climbs during a run, something
upstream (camera, network, or the detector) has regressed.

The mission layer is owned by the client, not the brain: while autonomous it
counts unique AprilTag IDs, and on reaching the "Tags to finish" target it flashes
the lights and hands control to the gamepad. The brain itself never stops or
celebrates — it just keeps a clean view of the other sub's tagged back.

Tuning is done live through the docked slider panel (TUNE), which writes
`strategy_gains.json`; the brain hot-reloads it, so a slider drag changes behavior
within a control loop or two. The same file drives the sim, so a gain set tuned in
Unity transfers to the sub unchanged.

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
python rov_server.py            # LAN
python rov_server.py --wifi     # WiFi
```

### Topside (laptop)

```
pip install -r requirements-client.txt
python rov_client.py                                  # video-only overlays
python rov_client.py --weights best.pt                # enable YOLO detection
python rov_client.py --weights best.pt --strategy strategy_full   # + autonomy brain
```

A matching `~/.rov_client_creds` holds the same network settings on the topside
machine.

### Training (any machine with a GPU, or CPU)

Keep torch/ultralytics out of your system Python with a venv, then train against a
Roboflow export:

```
python -m venv .venv && source .venv/bin/activate   # Windows: py -m venv .venv; .venv\Scripts\activate
pip install -U ultralytics
python train.py --data /path/to/roboflow/data.yaml
```

See the header of `train.py` for the NVIDIA CUDA-wheel note on Windows.

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

The harness measures total distance including the coast-to-rest glide, matching how
the real distance is measured (where the sub actually stops), so the two are
directly comparable.

---

## Running in the simulator

With the Unity scene playing (main sub carrying `SimBridge.cs`, both subs carrying
`RandomSpawn.cs`):

```
python run_sim.py --mock     # no Unity: scripted target, checks state transitions
python run_sim.py            # live: drives the main sub against the Unity target
python run_evade.py          # (optional, second terminal) the frozen target brain
```

Because `run_sim.py`'s packet path is byte-for-byte the real sub's, a strategy that
behaves in the sim behaves on the hardware.

---

## Notes on the light and camera

The light is controlled from the client with a simple on/off toggle, and a second
toggle flashes the light whenever an AprilTag is detected in the feed (the flash
overrides the manual state while a tag is in view). If the light does not respond,
confirm it is wired to a PCA9685 channel and use `light_finder.py` to identify that
channel, then set it in the server. The camera points via a servo on channel 15;
use `camera_aim.py` to aim it.

---

## Roadmap

The layers, in build order, and where they stand:

1. **Simulator tuning** — match the simulated sub to the real sub per axis using
   the fidelity loop above. Foundation and workbench in place; ongoing per axis.
2. **Vision model** — an object detector on the camera feed. In place:
   `train.py` produces weights, the client runs them live (in a separate process),
   and `render.py` checks a model off-sub.
3. **State-machine strategy** — a search / advance / orbit / scan behavior that
   turns a detection into control commands. In place: the shared `Strategy` brain
   runs in the sim (`run_sim.py`) and on the real sub (client autonomy), tuned
   through one hot-reloaded gains file.
4. **Sim/hardware protocol parity** — `SimBridge.cs` makes Unity answer the same
   UDP protocol as the real sub, so the identical topside controller drives either
   one with no code change. In place.
5. **Reinforcement learning** — a learned policy trained in simulation against the
   shared command path, dropped in behind the same `Strategy` interface. Future.

---

## Safety

The thruster loop fails safe: if commands stop arriving, a watchdog returns the
thrusters to neutral, and the Unity bridge does the same. The client sends a steady
idle keep-alive so the light and watchdog stay live even when no test is running,
and it neutralizes on quit. STOP aborts any test, autonomy run, or manual drive and
sends neutral immediately. The bench tools hold the thrusters neutral and leave the
camera servo untouched. When testing anywhere near the thrusters out of water, keep
clear of the propellers.
