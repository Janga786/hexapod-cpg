# Wiring summary — Arduino Mega 2560

The full pin map is also available as a single source-of-truth in
[`firmware/hexapod_full/config.h`](../firmware/hexapod_full/config.h).
This document reformats it for printable reference.

## Servos (18 channels, 3 per leg)

| Leg | Position    | Coxa | Femur | Tibia |
| --- | ----------- | ---: | ----: | ----: |
| 1   | front right |   19 |     8 |    23 |
| 2   | mid right   |   25 |    27 |    29 |
| 3   | back right  |   31 |    33 |    35 |
| 4   | back left   |   37 |    39 |    41 |
| 5   | mid left    |   43 |    45 |    47 |
| 6   | front left  |   49 |    51 |    53 |

All servos are powered through the SBEC, *not* from the Arduino's 5 V
rail.  Servo signal pins go directly to the Mega; servo grounds and
the SBEC ground are common to the Arduino ground.

## IMU (SparkFun ICM-20948)

| ICM pin | Mega pin | Notes |
| ------- | -------- | ----- |
| VDD     | 3V3      | not 5 V |
| GND     | GND      |         |
| SDA     | 20 (SDA) | shared with any other I²C devices |
| SCL     | 21 (SCL) |         |
| AD0     | tied high → I²C address `0x69` | (`IMU_AD0_VAL = 1` in `config.h`) |

## 3-bit binary command + confirm

| Function | Mega pin | Logic |
| -------- | -------- | ----- |
| Confirm  | A2       | rising → falling edge latches command |
| Bit 2 (MSB) | A3    | active low |
| Bit 1    | A4       | active low |
| Bit 0 (LSB) | A5    | active low |

All four pins use `INPUT_PULLUP`; tie an external switch to ground to
assert.

## Bump sensors

| Foot   | Mega pin | Reaction |
| ------ | -------- | -------- |
| Front-left  |  3 | back-up + rotate **right** |
| Front-right |  5 | back-up + rotate **left**  |

Normally-open switches; `INPUT_PULLUP` and pull to ground when pressed.

## Battery monitor

A resistor divider drops the LiPo bus voltage to the 0–5 V analog
range.  Read on `A0` and averaged over a 50-sample sliding window in
firmware.  Threshold at half scale flips the LED bar from green
(strong) to red (low) so the operator can see battery state at a
glance.

## LED bar (8 dual-colour LEDs)

| LED# | Red pin | Green pin |
| ---- | ------: | --------: |
| 1    |     22 |        24 |
| 2    |     26 |        28 |
| 3    |     30 |        32 |
| 4    |     34 |        36 |
| 5    |     38 |        40 |
| 6    |     42 |        44 |
| 7    |     46 |        48 |
| 8    |     50 |        52 |

The LED bar is shared between the gait-display function (when an
input button is held) and the battery monitor.

## Power tree

```
LiPo 7.4 V  ──┬──► SBEC ──► 5 V rail (servos, Arduino V_in)
              └──► resistor divider ──► A0 (battery monitor)
```

The Arduino is powered through `V_in`, *not* via USB, while running
on the LiPo.  Always disconnect the LiPo before plugging in USB for
firmware uploads.
