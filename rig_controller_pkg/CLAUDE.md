# CLAUDE.md — Rig Controller (`rig_controller_pkg/`)

This file is a recap of the CNC + Stewart platform + charge-mapping rig
controller: what each piece does, why it's built the way it is, and the
non-obvious bugs/decisions worth knowing before touching it again.

## What this system does

One Tkinter app, running on a Raspberry Pi, that drives an automated
experiment cycle:

**wiggle (Stewart platform tilts/orbits) → level → photograph the
particles → raster a charge probe over them (CNC) → re-home the CNC →
repeat**

...while building a live, on-screen heatmap of the probe's charge readings
and logging everything (samples + a photo per pass) to disk.

Three physically separate pieces of hardware are being coordinated at once:

| Hardware | Talks over | Controlled by |
|---|---|---|
| CNC router (CoreXY, BTT Pico, Klipper) | HTTP → Moonraker | `klipper_controller.py` |
| Stewart platform (6-leg parallel manipulator, BTT Octopus/STM32) | USB serial, custom text protocol | `platform_controller.py` |
| MCC118 DAQ HAT + electrostatic probe | daqhats Python lib | `daq_controller.py` |
| Pi Camera (imx477) | picamera2 | `camera_controller.py` |

## File-by-file

- **`main.py`** — entry point. `python3 main.py` launches everything.
- **`gui.py`** — the Tkinter app. Owns all the controller objects, lays out
  the panels (CNC / Stewart Platform / Auto Cycle / Charge Heatmap / Last
  Image), and is the only place background threads are allowed to touch
  Tkinter/matplotlib widgets (always via `root.after(...)`).
- **`klipper_controller.py`** — `KlipperController`. Sends G-code and reads
  position/homed-state from Moonraker's HTTP API.
- **`platform_controller.py`** — `PlatformController`. Talks to the
  Stewart platform over serial with its command set: `Z` (home from
  hand-set vertical), `H` (auto-home via hall sensors), `W` (start orbit),
  `S` (stop/level), `Q` (status query, added so scripts can poll
  "settled?" instead of guessing a sleep). Has `find_platform_port()` /
  `auto_connect()` which identify the Octopus by its USB descriptor
  (`F446`/`STMicroelectronics`, explicitly excluding `Klipper`/`rp2040` so
  it can never grab the CNC's Pico by mistake), with a hardcoded
  `/dev/ttyACM0` fallback.
- **`daq_controller.py`** — `DAQController`. Wraps the MCC118. Runs the
  scan **continuously on its own background thread**
  (`start_continuous()`), decoupled entirely from rasters or network
  calls — see "Why DAQ sampling is decoupled" below. Self-heals a stale
  "scan already active" hardware state left over from an unclean previous
  shutdown.
- **`camera_controller.py`** — `CameraController`. Lazy-imports
  `picamera2`. Single-still captures only (`create_still_configuration`),
  not video streaming.
- **`camera_view.py`** — `CameraView`. Tkinter panel showing the last
  captured image (needs Pillow).
- **`raster_gcode.py`** — `build_raster_gcode()`. Pure function, the
  raster-path math. Shared by the manual button and the auto-cycle so
  there's one source of truth.
- **`gcode_motion_model.py`** — `GCodeMotionModel`. Parses the exact raster
  G-code and models Klipper-style trapezoidal accel/cruise/decel motion
  per segment, so position at any timestamp can be **computed** instead of
  polled. Needs `max_accel` to match `printer.cfg` (currently `3000.0`,
  set as `CNC_MAX_ACCEL` in `gui.py`). Known approximation: treats each
  segment as an independent stop-start move, not accounting for Klipper's
  junction-deviation corner blending — small, localized error right at
  direction changes, negligible mid-segment.
- **`raster_recorder.py`** — `RasterRecorder`. Runs one raster pass:
  drains the DAQ's continuously-filling buffer (no network calls in this
  loop — see below), tags each sample with its *modeled* position, pushes
  to the heatmap's queue. Drift-checking (comparing modeled vs. real
  position) runs on its own separate thread so a slow Moonraker response
  can never stall sampling.
- **`live_heatmap.py`** — `LiveHeatmap`. ONE embedded matplotlib panel
  (not a new window per raster). Each sample paints a **disc** of radius
  `point_radius` (currently 15.75mm = half the probe's ~31.5mm diameter),
  not a single pixel — matters because the raster's line spacing (25mm)
  is smaller than the probe's footprint, so adjacent lines' coverage
  genuinely overlaps in reality. Axes are flipped (`invert_xaxis` +
  `invert_yaxis`) so the origin renders top-right, matching the physical
  rig's orientation. Color scale currently **auto-fits per raster**
  (fixed-range option exists via `vmin`/`vmax` but was tried and reverted
  — didn't suit "snapshot in time" viewing).
- **`data_export.py`** — `SessionCSVLogger`. **One CSV per app session**
  (not per raster) — every raster appends rows tagged with an
  incrementing `raster_number`. Also owns saving the pre-raster photo into
  a per-session `images/` subfolder, named by raster number
  (`peek_next_raster_number()` is used to pre-assign the number before the
  raster actually finishes, since the photo is taken *before* the raster
  runs but numbered to match the rows it'll produce). CSV columns:
  `raster_number, x_mm, y_mm, charge_v, timestamp, image_path`.
- **`cycle_controller.py`** — `CycleController`. The actual state machine:
  wiggle (timed) → level (fixed settle delay, see below) → raster
  (delegated to `RasterRecorder`, with `raster_start_cb`/`raster_done_cb`
  hooks the GUI uses for the camera + heatmap clear/save) → re-home CNC
  (every N cycles, not every single one — see below) → repeat.

## Why things are built the way they are (the non-obvious parts)

**DAQ sampling is fully decoupled from CNC/network calls.** Early version
called `klipper.get_position()` (an HTTP round-trip) inside the same tight
loop reading the DAQ. A slow/laggy Moonraker response stalled the whole
loop — wall-clock time kept advancing toward the raster's end while almost
no actual DAQ reads happened, producing far fewer samples than expected
(~101 instead of ~800 over 8s). Fixed by giving the DAQ its own dedicated
background thread (`start_continuous()`) that always reads at the
hardware's true pace, with `RasterRecorder` just draining that buffer in a
purely-local loop. Drift-checking (which does need the network) now runs
on a third, separate thread, so it can never block sampling again.
Verified with a deliberately-800ms-slow fake Klipper mock and still got
~97Hz.

**Position is computed, not polled, during a raster.** Same reason as
above — network calls are too slow/variable to do at DAQ sample rate.
`GCodeMotionModel` parses the exact G-code sent and computes expected
position from elapsed time + known accel physics instead.

**Auto-home sequencing.** Boot sequence is: CNC `G28` → wait for real
`homed_axes` confirmation → connect to the Stewart platform → send `H`
(hall-sensor auto-home). Each step gates the next; it won't try to
auto-home the platform if the CNC never confirmed home, or if the
platform never connected.

**Sensorless CNC homing = mechanical wear.** The CNC's `printer.cfg` does
StallGuard-based homing (X/Y intentionally drive into a hard stop, current
lowered to 0.6A during the move to soften impact). Re-homing every single
raster means repeated physical collisions. `CycleController` exposes
"re-home every N cycles" (GUI field) so this is tunable — defaults to
every cycle (N=1), safe to raise once you trust the mechanism isn't
missing steps.

**No real level-confirmation loop (yet).** There's no accelerometer on the
platform yet, so "confirm level" is currently just a fixed sleep
(`level_settle_delay_s`, GUI field). `PlatformController.wait_until_settled()`
(polls the firmware's `Q` command for a real "motors report settled"
answer) already exists and is ready to swap back in — and to extend with
a real roll/pitch check — once the accelerometer is wired up. The swap
point is commented directly in `cycle_controller.py`.

**Camera capture happens on a background thread, not the button
handler.** A still capture is a blocking hardware call; if it ran
synchronously in the manual-raster button's click handler, it'd freeze the
GUI momentarily. Both the manual and auto-cycle paths call
`_on_raster_start()` (which captures + saves the image) from a background
thread, then hop to the main thread only for the actual widget updates.

## Constants that must stay in sync with physical reality

- `CNC_MAX_ACCEL = 3000.0` in `gui.py` — must match `printer.cfg`'s
  `max_accel`. Feeds directly into the motion model; if it drifts out of
  sync, modeled positions will be subtly wrong.
- `sensor_radius = 15.75` in `gui.py` — half the probe's real ~31.5mm
  diameter. Used for the heatmap's disc-painting radius.
- DAQ config in `daq_controller.py`: `channel=1`, `sample_rate=100000.0`,
  `samples_per_read=1000`, `voltage_multiplier=10000.0`. **The
  `voltage_multiplier` was inherited from the original probe script as-is
  — its physical derivation (probe calibration factor? external amplifier
  gain?) was never independently verified.** Worth tracking down if the
  absolute magnitude of readings ever needs to be trusted quantitatively.

## Known open items / limitations

- Real-world sampling during a raster has been observed around ~70Hz
  against a ~100Hz theoretical rate (hardware confirms it locks to the
  full 100kHz/100Hz effective rate — the shortfall is downstream of that,
  most likely Pi CPU/GIL contention across the several threads now
  running). Not yet root-caused further; usable data either way.
- `voltage_multiplier`'s physical meaning is unverified (see above).
- No accelerometer-based level confirmation yet (see above) — platform
  "leveling" is currently trust-the-motors-plus-a-timer.
- Manual "Raster Area" button and the auto-cycle share the exact same
  `RasterRecorder`/heatmap/CSV/camera path, by design — no separate manual
  code path to drift out of sync.

## Dependencies

`pyserial`, `requests`, `numpy`, `matplotlib`, `pillow`. Plus, **system-level
only** (do not `pip install` these into an isolated venv — see below):
`picamera2` (needs `libcamera`, install via `apt`:
`sudo apt install python3-picamera2`), `daqhats` (MCC's own installer).

**If using a venv:** create it with `--system-site-packages` so it can see
the system-installed `picamera2`/`libcamera`/`daqhats`, e.g.:
```
python3 -m venv --system-site-packages venv
```
A plain venv without that flag, or a `pip install picamera2` inside an
isolated venv, will not work — `libcamera`'s Python bindings aren't a
normal pip package.

## Running it

```
python3 main.py
```

Requires Moonraker/Klipper already running and reachable (defaults to
`127.0.0.1:7125`). The Stewart platform auto-connects on launch; the DAQ
starts continuous polling on launch; the CNC homes, confirms, then the
platform connects and auto-homes, all before you touch anything.