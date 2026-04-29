# Hexapod Firmware

Three Arduino-Mega 2560 sketches live here, in increasing order of capability.

## `ps2_test/` — controller diagnostic
A 170-line diagnostic that brings up the PS2 receiver, prints joystick
positions to the serial console, and reports button-press events.
Useful as a first-light sanity check whenever you swap the gamepad,
re-route the receiver, or change the `PS2X_lib` configuration.

```
firmware/ps2_test/ps2_test.ino
```

Toolchain: Arduino IDE 2.x + the
[`PS2X_lib`](http://www.billporter.info/) library by Bill Porter.

## `legacy_ps2/` — full PS2-driven controller
The original, manually-piloted firmware: tripod / wave / ripple /
tetrapod gaits selected with the d-pad, joystick translate / rotate
modes, single-leg lift, and a calibration mode that pins every servo
at 90°.  Battery monitoring lights an 8-LED bar graph from the
analog-divider input.

```
firmware/legacy_ps2/legacy_ps2.ino
```

This sketch is preserved unchanged because it is the simplest path to
a working hexapod — useful for smoke-testing a new build before
flashing the autonomous firmware.

## `hexapod_full/` — autonomous firmware (main)
The flagship firmware.  A 3-bit binary command interface (latched on
the rising edge of a confirm pin) selects between eight behaviours:

| Code | Behaviour |
| ---- | --------- |
| `000` | Idle (home pose) |
| `001` | Wave hello (front-right leg) |
| `010` | **Forward walk** with IMU heading hold + bump avoidance |
| `011` | Dance: side-to-side sway with swagger |
| `100` | Body pump (~3 Hz vertical bounce) |
| `101` | Twist (yaw rotation with double-frequency bounce) |
| `110` | Stomp (alternating tripod groups) |
| `111` | Full routine (cycles through 1, 4, 3, 5, 6, 4 every 3 s) |

```
firmware/hexapod_full/hexapod_full.ino
firmware/hexapod_full/config.h         ← all field-tunable constants
```

### Heading hold

Code `010` engages a closed-loop yaw controller built around a
SparkFun ICM-20948 IMU (gyro + accelerometer + magnetometer). On
power-up the firmware:

1. Reads the magnetometer 20 times to seed `current_heading`.
2. Calibrates the gyro Z bias by averaging 50 samples over 1 s while
   the robot is held still.
3. Latches `desired_heading = current_heading` the moment the walk
   command is selected.

Each 20 ms control frame:

- Integrates `gyrZ - bias` for the time-step, producing a high-rate
  heading prediction.
- Reads the magnetometer for an absolute heading reference.
- Fuses them with a complementary filter:
  `current_heading = pred + (1 - α)·(mag − pred)`.
- Drives a P-controller on the heading error:
  `commandedR = clamp(K_p · (desired − current), −127, +127)`.

This compensates for asymmetric servo torque, terrain bias, and
bump-avoidance recoveries.  Re-tune the gains via `HEADING_KP` and
`COMP_ALPHA` in `config.h`.

### Bump avoidance

Two foot-mounted bump switches (front-left, front-right) are sampled
every frame.  When one is pressed during forward walk:

```
mode 1 (walk) ──bump──► mode 10 (backup, 0.8 s, ramped) ──► mode 11 (turn 90°, 1.0 s) ──► mode 1
```

The turn direction is the *opposite* of the bumped foot (a left-foot
bump means an obstacle to the front-left, so we rotate right to clear
it).  Re-triggers are debounced for 1.5 s so a repeated brush against
the same obstacle doesn't loop on itself.

### Performance routines

`action_wave`, `action_dance_sway`, `action_body_pump`,
`action_twist`, `action_stomp` and the `action_routine` aggregator
implement open-loop time-varying foot-position offsets to produce
expressive idle motions.  All are pure functions of `perf_tick`
(the frame counter), so duration / amplitude tuning is straightforward.

### Frame timing

A fixed 20 ms control loop (`FRAME_TIME_MS = 20` in `config.h`) drives
the entire system at 50 Hz.  All gait integrators, the heading filter,
and the bump-debounce timer are clocked off this single tick.

### Tuning

Every field-tunable constant lives in `config.h`, grouped by purpose
and annotated with sensible ranges.  In a typical session you might
adjust:

- `HEADING_KP` — heading-controller aggressiveness (typical 1.0–5.0).
- `COMP_ALPHA` — gyro vs. magnetometer trust (typical 0.90–0.98).
- `BUMP_BACKUP_FRAMES` / `BUMP_TURN_FRAMES` — recovery timing.
- `STEP_HEIGHT_MULT` — swing-arc lift height (0.5–2.0).

`COXA_CAL`, `FEMUR_CAL`, `TIBIA_CAL` are per-axis offsets; re-tune by
running the `legacy_ps2` sketch in calibration mode (Select button)
and trimming until the robot is mechanically straight.

## Hardware

| Component | Notes |
| --- | --- |
| Arduino Mega 2560 | for 18 PWM servo channels + analog & digital I/O |
| 18 × hobby servos (~7-9 kg·cm) | three per leg: coxa, femur, tibia |
| SparkFun ICM-20948 | I²C IMU, AD0 = high → 0x69 |
| 2 × foot-pad bump switches | normally-open, pulled up |
| 8 × dual-colour LEDs | battery-status bar graph |
| 7.4 V LiPo + SBEC | power; analog divider on `BATT_VOLTAGE_PIN` |

The pin assignments are exhaustively documented in
[`hexapod_full/config.h`](hexapod_full/config.h).  See
[`docs/wiring.md`](../docs/wiring.md) for a printable summary.
