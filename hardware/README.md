# Hardware

Mechanical CAD, bill-of-materials, and printable-parts list for the
hexapod chassis.  The printed skeleton is Mark W's open-source
[hexapod design](https://www.projectsofdan.com/), and the print list
below covers his skeleton's parts.  The SolidWorks parts (foot models with sensor mounts,
a dome cover, a buck-converter holder) were added for this build.

## Layout

```
hardware/cad/
├── stl/                # 22 STL files (everything you'd 3-D print)
├── sldprt/             # 12 SolidWorks part files (editable source)
└── assembly/           # full-robot assembly + Fusion archive + STEP export
```

The full body is also exported as a single
[`hexapod_assembly.step`](cad/assembly/hexapod_assembly.step) file
which most CAD packages can open directly without a SolidWorks
licence.  An `Foot&Leg Assembly.SLDASM` is included for the leg
sub-assembly only.  The Fusion 360 archive
(`Hexapod.f3z`) is the working source.

## Printed parts

The detailed printing instructions are in
[`docs/printing_instructions.docx`](../docs/printing_instructions.docx).
Summary (124 printed parts in total):

### PLA, 30 % infill, **no support**
- 1 × Body Bottom Plate
- 6 × Body Riser
- 1 × Body Top Plate
- 12 × Femur Bracket End Cap
- 12 × Femur Bracket
- 18 × Servo Bearing Center
- 6 × Tibia Base Plate
- 6 × Tibia Bracket End Cap
- 6 × Tibia Bracket
- 6 × Tibia Foot Plate
- 6 × Tibia Side 1
- 6 × Tibia Side 2  *(identical mirror; print with opposing sides down)*
- 6 × Tibia Spacer Tube

### PLA, 30 % infill, **with support**
- 1 × Receiver Holder
- 1 × SBEC Holder
- 18 × Servo Mount
- 6 × Wire Guide

### TPU, 10 % infill, **no support**
- 6 × Tibia Foot Bumper

## Electronics

| # | Component | Notes |
| - | --------- | ----- |
| 1 | Arduino Mega 2560 | the brain — 18 PWM channels + abundant digital I/O |
| 18 | Hobby servo (≥ 7 kg·cm) | three per leg (coxa / femur / tibia) |
| 1 | SparkFun ICM-20948 | 9-DoF IMU (used for gyro + magnetometer) |
| 2 | Foot-pad bump switch | normally-open, pulled up to 5 V |
| 8 | Dual-colour LED + 1 kΩ resistors | battery-status bar graph |
| 1 | LiPo battery (2S or 3S, ≥ 5 Ah) | main power |
| 1 | SBEC (5 V, ≥ 6 A) | servo / Arduino power regulator |
| 1 | PS2 receiver + DualShock controller | (only used by `legacy_ps2.ino`) |

See [`docs/wiring.md`](../docs/wiring.md) for the exhaustive
pin-by-pin connection list.
