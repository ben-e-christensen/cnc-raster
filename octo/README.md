# Octopus Stewart Platform Firmware - Stage 4

Bring-up project for driving your rotary Stewart platform's six NEMA17
steppers from a BigTreeTech Octopus V1.1 (STM32F446ZET6), using PlatformIO
+ the Arduino/STM32Duino framework instead of Klipper or Marlin.

This stage adds real inverse kinematics: send a target platform pose over
serial, and all six legs move to reach it. All the geometry (base pivots,
shaft directions, platform attachment points, the confirmed motor<->hole
pairing, the 60-degree assembly twist, the rod length and crank throw) lives
in `kinematics.h` - see that file's header comment for the full derivation.

## Hard-won lessons baked into this version (from stages 1-3)

1. **`driver.toff(4)` is mandatory** - TMC2209's output stage defaults to
   disabled at power-up; only a UART write turns it on.
2. **Use `SoftwareSerial`, not `HardwareSerial`** for the per-driver UART
   pins - none of them are real hardware USART pins on this chip.
3. **Each driver socket's jumper block must be set to UART mode, not SPI
   mode** - TMC2209 has no SPI interface at all.

## Serial commands (115200 baud)

- `Z` - home/zero. **Physically position the platform level, centered, and
  at the neutral height (~171.74mm) before sending this.** It tells the
  firmware "this is the neutral pose" and computes each leg's reference
  crank angle from it - everything after is tracked relative to this.
- `P x y z roll pitch yaw` - command a new target pose. Position in mm,
  relative to the neutral position (so `P 0 0 0 0 0 0` returns to neutral),
  angles in degrees. Example: `P 0 0 0 5 0 0` tilts 5 degrees of roll.
  Sending a `P` command cancels any active orbit (`W`).
- `W tilt_deg [period_sec]` - orbital tilt: holds a constant tilt magnitude
  while continuously rotating the tilt *direction* around a full circle -
  like swirling a marble around the rim of a bowl, rather than tipping the
  platform once and holding still. `period_sec` is how long one full
  revolution takes (default 4 seconds if omitted). Example: `W 10 3` orbits
  a 10-degree tilt once every 3 seconds. **12.0 degrees is the largest tilt
  magnitude that stays reachable at every point around the full circle**,
  confirmed via `workspace_sweep.py`'s `max_safe_orbit_radius()` - go
  higher and the platform will stutter/hold at whichever phase angles push
  past the envelope, since those particular poses come back unreachable.
- `S` - stop an active orbit and hold the current position.

You must send `Z` before any `P` or `W` command will be accepted.

## Known gotcha: float parsing in `sscanf`

Some minimal embedded libc configurations disable float support in
`scanf`/`printf` by default to save flash space. If the `P` command's
`sscanf` silently returns fewer than 6 parsed values even with correctly
formatted input, this is almost certainly why - not a firmware logic bug.
Fix: add `-u _scanf_float` (newlib) to your linker flags. Worth checking
early, since the symptom ("parse error" on obviously-correct input) doesn't
point at the real cause on its own.

## Motion model

Each leg moves toward its target step position at a fixed max rate (500
steps/sec) - no acceleration/deceleration profile yet. Fine for a
slow-moving balancing platform; revisit if you want fast, smooth pose
changes later.

## One-time setup

1. Install PlatformIO Core (CLI only, no IDE/extension required):
   ```
   pip install platformio
   ```
2. From this project's folder, put the Octopus into DFU bootloader mode:
   - Set the `J75` BOOT0 jumper.
   - Press the board's reset button once.
3. Build and upload:
   ```
   pio run -e octopus -t upload
   ```
4. Remove the BOOT0 jumper and reset the board again to run normally.
5. Open the serial monitor:
   ```
   pio device monitor -b 115200
   ```
6. Physically level the platform at its neutral height, send `Z`, then try
   `P 0 0 0 5 0 0` to test a small roll motion.

## Project layout

- `platformio.ini` - build config; targets the `octopus` environment.
- `boards/genericSTM32F446ZE.json` - custom board definition (Arduino
  framework support for this exact chip/flash/RAM combination).
- `STM32F446ZETx_FLASH.ld` - linker script STM32Duino doesn't ship for this
  variant.
- `src/kinematics.h` - all geometry constants and the per-leg IK solve.
- `src/main.cpp` - motor/UART setup, homing, serial commands, motion loop.

## Next stages (not yet in this project)

- Tune `TEST_CURRENT_MA` (currently a conservative 600mA) up to match your
  NEMA17's actual rated current.
- Acceleration/deceleration profiling for smoother, faster pose changes.
- A richer command set (e.g. smooth interpolated moves between poses,
  rather than each leg independently racing to its own target at a fixed
  rate - fine for small moves, but large simultaneous multi-axis moves
  will currently arrive at slightly different times per leg).


