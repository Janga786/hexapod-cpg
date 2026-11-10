<div align="center">

# Hexapod CPG: an 18-DoF field hexapod and a Kuramoto gait simulator

[![CI](https://github.com/Janga786/hexapod-cpg/actions/workflows/ci.yml/badge.svg)](https://github.com/Janga786/hexapod-cpg/actions/workflows/ci.yml)
[![Tests](https://img.shields.io/badge/tests-34%20passing-brightgreen.svg)](#testing)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

<img src="figures/hexapod_hero.jpg" alt="The hexapod robot on sand during the challenge" width="92%"/>

<sub>The team's hexapod on sand at Great Sand Dunes during the Colorado Robotics Challenge.</sub>

</div>

This repository has two related parts. The first is the 18-DoF hexapod that
our four-person Fort Lewis College team built and fielded for NASA Space
Grant's Colorado Robotics Challenge (Oct 2025 – Apr 2026, advisor
Dr. Charles Hakes). I led the team. The second is a Python central pattern
generator (CPG) simulator I wrote afterwards, in which six Kuramoto-coupled
phase oscillators produce wave, ripple and tripod gaits. **The CPG runs in
simulation only. The fielded robot walked under the Arduino firmware.**

Project page: [janga786.github.io/projects/hexapod-nasa-challenge](https://janga786.github.io/projects/hexapod-nasa-challenge/)

## Attribution

| Who | What |
| :--- | :--- |
| **Me (Jangara Bliss)** | Led the four-person team. Designed the electrical system: Arduino Mega 2560, SparkFun ICM-20948 9-DoF IMU, foot bump switches, LiPo + SBEC power ([`docs/wiring.md`](docs/wiring.md)). Extended the firmware in [`firmware/hexapod_full/`](firmware/hexapod_full/) with gyro-magnetometer heading hold, gyro-bias calibration at startup, and bump avoidance. Later, on my own, wrote the Kuramoto CPG simulator in [`sim/`](sim/), its figures and animations, and its 34-test suite. |
| **The team (four students, including me)** | Built, assembled and operated the physical robot in the field. Teammates' challenge-era code lives in the team's own repositories under another GitHub account. |
| **Mark W (open-source original)** | The 3-D-printed chassis skeleton and the original PS2-controller firmware ([projectsofdan.com](https://www.projectsofdan.com/?cat=4)). [`firmware/legacy_ps2/`](firmware/legacy_ps2/) and [`firmware/ps2_test/`](firmware/ps2_test/) are his sketches. [`firmware/hexapod_full/`](firmware/hexapod_full/) is his firmware modified and extended for this robot (see its header). |
| **Other references and libraries** | The leg geometry and IK in the original firmware follow the external references credited in its source header. The IMU and PS2 drivers are the SparkFun ICM-20948 library and Bill Porter's PS2X library. |

## The challenge and outcome

The robot walked outdoors on loose sand and small rocks at Great Sand Dunes
(photos [below](#the-robot-in-the-field)). In walk mode the firmware holds a
compass heading with a gyro-magnetometer complementary filter and a
P-controller, and backs up and turns when a front foot hits an obstacle.
This repository does not record a placement, a timed course result, or a
repeatable terrain success rate, so the field work is evidence of a working,
integrated robot rather than a measured locomotion result.

## Run the simulator

```bash
git clone https://github.com/Janga786/hexapod-cpg.git
cd hexapod-cpg
python -m venv .venv && source .venv/bin/activate

make dev           # editable install + dev tools
make test          # 34 tests, under 1 s
make sim           # live 4-panel dashboard
make gallery       # regenerate every figure and animation (about 3 minutes)
```

Dependencies: `numpy`, `matplotlib`, `pillow`. CI tests Python 3.10, 3.11
and 3.12 on Ubuntu and macOS.

<div align="center">
<img src="animations/cpg_dashboard_teaser.gif" alt="CPG simulator dashboard" width="92%"/>

<sub>Simulator dashboard: 3-D body with foot trails, phase wheel, rolling gait diagram, status panel.</sub>
</div>

## Limitations

- The CPG has never driven the physical robot. It is a kinematic simulation
  only: no dynamics, contact or terrain model.
- In the CPG, the target phase offsets are designed. A smoothstep in ω blends
  a wave offset matrix into a tripod offset matrix, and "ripple" is the
  intermediate blend. The oscillator coupling makes the legs lock onto those
  targets from random starting phases; the gait patterns themselves are not
  discovered by the network.
- The firmware has no automated tests, and no field logs (heading error,
  bump events, distance walked) are kept in this repository.
- Not done: driving the real robot from the CPG over serial, and feeding foot
  contact back into the oscillators.

---

## Contents

- [Attribution](#attribution)
- [The challenge and outcome](#the-challenge-and-outcome)
- [Run the simulator](#run-the-simulator)
- [Limitations](#limitations)
- [Highlights](#highlights)
- [The robot in the field](#the-robot-in-the-field)
- [Simulator gallery](#simulator-gallery)
- [Mathematical foundations](#mathematical-foundations)
- [Repository layout](#repository-layout)
- [Python example](#python-example)
- [Reproducing every figure & animation](#reproducing-every-figure--animation)
- [Firmware overview](#firmware-overview)
- [Hardware overview](#hardware-overview)
- [Testing](#testing)
- [References](#references)
- [Citation](#citation)
- [License](#license)

---

## Highlights

| Capability | Implementation | Notes |
| :--- | :--- | :--- |
| 6-oscillator Kuramoto CPG with smooth wave-to-tripod blending | `hexapod_cpg.cpg.CPGNetwork` | Tested: high ω locks to coherence > 0.85 within 5 s; recovers from a π perturbation |
| 18-DoF analytical inverse kinematics with reach and servo-limit checks | `hexapod_cpg.geometry.ik` | Tested: IK→FK round trip exact at the neutral pose for every leg |
| Foot-trajectory generator (cosine stance, smoothstep swing, sine-arc lift) | `hexapod_cpg.cpg.CPGNetwork._feet_from_phases` | Duty factor 0.70 to 0.62 over ω = 1–5 |
| 4-panel matplotlib dashboard (3-D, phase wheel, gait diagram, status) | `hexapod_cpg.viz.CPGViz` | 30 fps target (33 ms frame interval) |
| Heading hold on the robot (complementary filter + P-controller) | `firmware/hexapod_full/` | Gyro-bias calibration at startup; `K_p` and `α` in `config.h`; no automated tests |
| Bump-avoidance state machine (walk, back up, turn, walk) | `firmware/hexapod_full/` | 1.5 s debounce, ramped speed transitions |
| 6 scripted motion routines (wave, sway, pump, twist, stomp, full) | `firmware/hexapod_full/` | Open-loop, time-varying foot offsets |
| Mechanical CAD | `hardware/cad/` | 22 STL, 12 SolidWorks parts, STEP and Fusion archive; 124 printed parts per the print list |

The 34 pytest tests run on every push to `main` and on pull requests, across
Python 3.10/3.11/3.12 on Ubuntu and macOS, via GitHub Actions. CI also runs
`ruff` and a smoke test of four rendering scripts.

---

## The robot in the field

<table>
<tr>
<td align="center" width="50%">
<img src="figures/hexapod_walking_obstacles.jpg" alt="Walking through obstacles" width="100%"/><br/>
<sub><b>In motion.</b>  The robot on loose sand and small rocks. The flat panels on the front legs were added in the field.</sub>
</td>
<td align="center" width="50%">
<img src="figures/hexapod_in_field.jpg" alt="Three-quarter view on sand" width="100%"/><br/>
<sub><b>On the dunes.</b>  Three-quarter rear view on rippled sand.</sub>
</td>
</tr>
<tr>
<td align="center" colspan="2">
<img src="figures/hexapod_workbench.jpg" alt="Side view, on a table" width="60%"/><br/>
<sub><b>Between runs.</b>  Three servos per leg (coxa, femur, tibia), electronics on top, blue TPU foot bumpers.</sub>
</td>
</tr>
</table>

---

## Simulator gallery

<table>
<tr>
<td align="center" width="50%">
<img src="animations/walk_tripod.gif" alt="Tripod gait" width="100%"/><br/>
<sub><b>Tripod gait.</b>  ω = 5.0 rad/s.  Two anti-phase groups of three legs alternate; this is the textbook fast hexapod gait.</sub>
</td>
<td align="center" width="50%">
<img src="animations/walk_wave.gif" alt="Wave gait" width="100%"/><br/>
<sub><b>Wave gait.</b>  ω = 1.0 rad/s.  60° front-to-back propagation at a high duty factor (β ≈ 0.70 — roughly four legs in stance at any moment).</sub>
</td>
</tr>
<tr>
<td align="center">
<img src="animations/walk_ripple.gif" alt="Ripple gait" width="100%"/><br/>
<sub><b>Ripple gait.</b>  ω = 3.0 rad/s.  The intermediate blend between the wave and tripod offsets; about four legs in stance at a time.</sub>
</td>
<td align="center">
<img src="animations/phase_wheel_lockin.gif" alt="Phase-wheel convergence" width="100%"/><br/>
<sub><b>Phase-wheel lock-in.</b>  Six dots on the unit circle, scattered at t = 0, lock into the tripod pattern within a few seconds.</sub>
</td>
</tr>
</table>

<table>
<tr>
<td align="center" width="33%">
<img src="figures/coupling_graph.png" alt="Coupling graph" width="100%"/><br/>
<sub><b>Coupling graph.</b>  Top-down view of the body with ipsilateral / contralateral / diagonal CPG edges.</sub>
</td>
<td align="center" width="33%">
<img src="figures/gait_comparison.png" alt="Gait diagrams" width="100%"/><br/>
<sub><b>Gait diagrams.</b>  Stance/swing patterns at ω = 1, 3, 5 rad/s.  Wave / ripple / tripod emerge from <em>the same</em> CPG.</sub>
</td>
<td align="center" width="33%">
<img src="figures/oscillator_timeseries.png" alt="Phase timeseries + coherence" width="100%"/><br/>
<sub><b>Convergence.</b>  Random initial phases lock into a tripod gait; pattern coherence (green) climbs to 1, while the global Kuramoto order (orange) misleadingly stays low — antiphase tripod groups average out.</sub>
</td>
</tr>
<tr>
<td align="center">
<img src="figures/foot_trajectories.png" alt="Foot trajectories" width="100%"/><br/>
<sub><b>Foot trajectories.</b>  One full cycle of (x, y, z) foot path per leg, rendered for each gait; the smoothstep swing arc and the cosine stance segment are directly visible.</sub>
</td>
<td align="center">
<img src="figures/omega_sweep.png" alt="Omega sweep" width="100%"/><br/>
<sub><b>ω-sweep.</b>  Duty factor and the smoothstep blend coefficient α(ω) over the entire operating range, with gait labels coloured.</sub>
</td>
<td align="center">
<img src="figures/leg_workspace.png" alt="Leg workspace" width="100%"/><br/>
<sub><b>Leg workspace.</b>  Reachable region in the (x, z) plane for one leg, with the neutral foot, the stance segment and the swing arc overlaid.</sub>
</td>
</tr>
</table>

---

## Mathematical foundations

### 1. Kuramoto coupled-oscillator network

Each leg is one phase oscillator $\varphi_i \in [0, 2\pi)$ and the six are
coupled by a directed graph that encodes ipsilateral, contralateral, and
diagonal interactions:

$$
\dot{\varphi_i} = \omega + \sum_{j} K_{ij}\,\sin(\varphi_j - \varphi_i - \theta_{ij}(\omega)) + \eta_i
$$

with $\eta_i \sim \mathcal{N}(0, \sigma)$ a small phase noise injection so the
lock-in to the target pattern is visible from random initial conditions.

The phase-offset matrix $\theta_{ij}(\omega)$ smoothly interpolates between
two target patterns:

- $\theta^{\mathrm{wave}}$ — even 60° spacing front-to-back
  (LF=0°, LM=60°, LB=120°, RF=180°, RM=240°, RB=300°);
- $\theta^{\mathrm{tripod}}$ — two anti-phase groups
  (group A: LF, LB, RM at 0°; group B: RF, LM, RB at 180°).

The blend coefficient is $\alpha(\omega) = \mathrm{smoothstep}((\omega - 1) / 4)$, giving a
wave gait at low $\omega$ and a tripod gait at high $\omega$, with
ripple-like patterns in between.

Integration is **explicit midpoint (RK2)** at the visualiser frame rate
($\sim 30 \mathrm{Hz}$); stable for the coupling strengths in use and substantially
more accurate than forward Euler at the same cost.

### 2. Foot trajectory generator

Phase $\varphi_i$ partitions the cycle into a *stance* segment of duration
$\beta\,T$ (where $\beta$ is the duty factor and $T = 2\pi / \omega$) and a
*swing* segment of duration $(1-\beta)T$:

$$
\mathbf p_i(\varphi_i) =
\begin{cases}
\mathbf p^{\mathrm{neutral}}_i + \mathbf s_i\,(\tfrac{1}{2} - t_{\mathrm{st}}) & \text{(stance)} \\
\mathbf p^{\mathrm{neutral}}_i + \mathbf s_i\,(\mathrm{smoothstep}(t_{\mathrm{sw}}) - \tfrac{1}{2}) + h\,\sin(\pi t_{\mathrm{sw}})\,\hat z & \text{(swing)}
\end{cases}
$$

The duty factor itself decreases with $\omega$, modelling the empirical
observation that fast hexapods spend less time on each foot —
$\beta(\omega) = \mathrm{clamp}(0.70 - 0.08(\omega - 1)/4,\ 0.50,\ 0.72)$.

The stride vector $\mathbf s_i$ blends a translational component (set by the
walk direction) with a rotational component (tangent to the leg's mount
radius), so a single CPG drives both linear and angular locomotion.

### 3. Closed-form 3-DoF leg kinematics

For a target foot position $\mathbf p = (x, y, z)$ in the body frame and a
coxa-mount at $(m_x, m_y)$:

$$
\mathrm{coxa}    = \operatorname{atan2}(\Delta y, \Delta x) - \theta_m
$$

$$
\rho             = \sqrt{\Delta x^2 + \Delta y^2} - \ell_c
$$

$$
D                = \sqrt{\rho^2 + z^2}
$$

$$
\mathrm{femur}   = \arccos\!\left(\frac{D^2 + \ell_f^2 - \ell_t^2}{2 D \ell_f}\right) + \operatorname{atan2}(z, \rho)
$$

$$
\mathrm{tibia}   = \arccos\!\left(\frac{\ell_f^2 + \ell_t^2 - D^2}{2 \ell_f \ell_t}\right) - 90^\circ - \beta_{\mathrm{off}}
$$

with reach checks against the femur+tibia 2-link chain and servo-range
checks before the result is marked valid.

### 4. Heading-hold controller (firmware)

The Arduino firmware runs a fixed-rate (50 Hz) yaw P-controller fed by
a complementary filter that fuses the IMU's gyro Z and magnetometer:

$$
\hat\psi[k+1] = \hat\psi[k] + (g_z - b_z)\,\Delta t + (1 - \alpha)\,\bigl(\psi_{\mathrm{mag}} - (\hat\psi[k] + (g_z - b_z)\,\Delta t)\bigr)
$$

$$
u[k] = \mathrm{clip}(K_p\,(\psi^* - \hat\psi[k]),\ -127,\ +127)
$$

where $b_z$ is the resting gyro Z bias auto-calibrated at startup and
$\psi^*$ is the heading latched at the moment the walk command is
issued.

---

## Repository layout

```
hexapod-cpg/
├── sim/                       # Python CPG simulation (the package)
│   ├── hexapod_cpg/           # importable Python package
│   │   ├── __init__.py        # public API
│   │   ├── geometry.py        # 18-DoF IK + body geometry
│   │   ├── cpg.py             # 6-oscillator Kuramoto network
│   │   ├── gait.py            # wave / ripple / tripod taxonomy
│   │   ├── colors.py          # palette + ω → colour map
│   │   ├── viz.py             # 4-panel matplotlib dashboard
│   │   └── py.typed
│   ├── scripts/               # one-shot rendering and demo drivers
│   │   ├── run_interactive.py            # launch the dashboard
│   │   ├── render_coupling_graph.py
│   │   ├── render_omega_sweep.py
│   │   ├── render_workspace_map.py
│   │   ├── render_oscillator_timeseries.py
│   │   ├── render_gait_comparison.py
│   │   ├── render_foot_trajectories.py
│   │   ├── render_phase_wheel_animation.py
│   │   ├── render_3d_walk_animation.py
│   │   └── render_all.py                 # regenerates the entire gallery
│   └── tests/                 # 34 pytest tests
│
├── firmware/                  # Arduino-Mega 2560 firmware
│   ├── hexapod_full/          # IMU heading-hold + bump-avoid + dance routines
│   │   ├── hexapod_full.ino
│   │   └── config.h           # all field-tunable constants
│   ├── legacy_ps2/            # original PS2-controller firmware (multi-gait)
│   │   └── legacy_ps2.ino
│   ├── ps2_test/              # diagnostic for the PS2 receiver
│   │   └── ps2_test.ino
│   └── README.md              # firmware architecture, gain tuning, state machines
│
├── hardware/                  # mechanical CAD
│   ├── cad/
│   │   ├── stl/               # 22 STL files (printable parts)
│   │   ├── sldprt/            # 12 SolidWorks part files
│   │   └── assembly/          # full-robot assembly + STEP + Fusion archive
│   └── README.md              # printed-parts list + electronics BOM
│
├── docs/                      # supplemental documentation
│   ├── wiring.md              # full Arduino-Mega pin map
│   └── printing_instructions.docx
│
├── figures/                   # static PNGs emitted by the renderers
├── animations/                # animated GIFs + dashboard MP4
├── pyproject.toml             # PEP 517/518 build + tooling
├── requirements.txt
├── requirements-dev.txt
├── Makefile                   # `make install`, `make test`, `make gallery`
├── .github/workflows/ci.yml   # multi-OS / multi-Python CI
├── CITATION.cff
├── LICENSE                    # MIT
└── README.md                  # you are here
```

---

## Python example

```python
import numpy as np
from hexapod_cpg import CPGNetwork

cpg = CPGNetwork(omega=5.0, rng=np.random.default_rng(42))
cpg.set_walking(direction_deg=0.0)        # walk forward

for _ in range(int(5.0 / 0.02)):           # 5 s @ 50 Hz
    feet = cpg.step(0.02)

print(f"locked-in coherence : {cpg.phase_coherence():.2f}")
print(f"LF foot position    : {feet['LF']}")
```


---

## Reproducing every figure & animation

```bash
make gallery        # one shot — regenerates /figures and /animations
```

Or call the renderers individually:

| Renderer | Outputs |
| :--- | :--- |
| `python sim/scripts/render_coupling_graph.py` | `figures/coupling_graph.png` |
| `python sim/scripts/render_omega_sweep.py`    | `figures/omega_sweep.png` |
| `python sim/scripts/render_workspace_map.py`  | `figures/leg_workspace.png` |
| `python sim/scripts/render_oscillator_timeseries.py` | `figures/oscillator_timeseries.png` |
| `python sim/scripts/render_gait_comparison.py`| `figures/gait_comparison.png` |
| `python sim/scripts/render_foot_trajectories.py` | `figures/foot_trajectories.png` |
| `python sim/scripts/render_phase_wheel_animation.py` | `animations/phase_wheel_lockin.gif` |
| `python sim/scripts/render_3d_walk_animation.py` | `animations/walk_{wave,ripple,tripod}.gif` |

All renderers use fixed random seeds.

---

## Firmware overview

`firmware/hexapod_full/hexapod_full.ino` is the flagship sketch.  A
3-bit binary command interface (latched on the rising edge of a confirm
pin) selects between eight behaviours; the truth table lives in
[`firmware/README.md`](firmware/README.md).  The `010` command engages
forward walk with an IMU heading-hold P-controller, a complementary
filter for gyro/mag fusion, and a bump-avoidance state machine that
backs up + rotates 90° away from any obstacle the front feet hit.

All field-tunable constants — `HEADING_KP`, `COMP_ALPHA`, the bump
timings, and per-axis servo-calibration offsets — live in
[`firmware/hexapod_full/config.h`](firmware/hexapod_full/config.h),
not scattered through the `.ino`.

The legacy `legacy_ps2.ino` sketch is preserved as the simplest path
to a working hexapod (PS2-controller piloted; tripod / ripple / wave /
tetrapod gaits selectable from the d-pad), useful for first-light
testing of a new build.

---

## Hardware overview

The chassis is 3-D printed (PLA, with TPU foot bumpers) from Mark W's
open-source skeleton. The 22 STL files, the SolidWorks parts, a full-robot
STEP export and a Fusion 360 archive are under [`hardware/cad/`](hardware/cad/).
The print list in [`docs/printing_instructions.docx`](docs/printing_instructions.docx)
comes to 124 printed parts.

Electronics:

- 1 × Arduino Mega 2560
- 18 × hobby servos (~7-9 kg·cm)
- 1 × SparkFun ICM-20948 9-DoF IMU
- 2 × foot-pad bump switches
- 8 × dual-colour LEDs (battery-status bar)
- 1 × LiPo + SBEC

Full pinout and power tree: [`docs/wiring.md`](docs/wiring.md).

---

## Testing

```bash
make test                                       # 34 tests, < 1 s
python -m pytest sim/tests/test_cpg.py -v       # one module, verbose
python -m pytest --cov=hexapod_cpg              # with coverage
```

The test suite covers:

- IK reachability + IK→FK round-trip for every leg at neutral pose
- Workspace boundary handling (unreachable targets marked invalid)
- Coupling-matrix invariants (symmetry, no self-coupling)
- Tripod target offsets are exactly π for every cross-group pair
- High-ω lock-in convergence to coherence > 0.85 in < 5 s
- Low-ω wave-gait coherence > 0.6 in < 8 s
- Pattern coherence drops on a π perturbation and recovers afterwards
- Stop / start / clamp behaviour on the speed setter
- Smoothstep gait-blend monotonicity and clamping
- Duty-factor monotonic decrease with ω

---

## References

1. Y. Kuramoto. *Chemical Oscillations, Waves, and Turbulence*. Springer, 1984.
2. A. Ijspeert. "Central pattern generators for locomotion control in animals and robots: a review." *Neural Networks* 21(4), 2008.
3. J. Schmitz et al. "Behaviour-based modelling of hexapod locomotion: linking biology and technical application." *Arthropod Structure & Development* 33, 2004.
4. R. D. Quinn & R. E. Ritzmann. "Construction of a hexapod robot with cockroach kinematics benefits both robotics and biology." *Connection Science* 10(3-4), 1998.
5. Mark W., *Hexapod skeleton design*, [projectsofdan.com](https://www.projectsofdan.com/?cat=4) (mechanical design + PS2 firmware origin).
6. SparkFun Electronics. *ICM-20948 Arduino Library*.
7. Bill Porter. *PS2X Arduino Library*. <http://www.billporter.info/>

---

## Citation

If you reference this code, please cite via the
[`CITATION.cff`](CITATION.cff) file or BibTeX below:

```bibtex
@software{janga2026hexapod,
  author  = {Bliss, Jangara},
  title   = {Hexapod CPG: an 18-DoF field hexapod and a Kuramoto gait simulator},
  year    = {2026},
  version = {1.0.0},
  url     = {https://github.com/Janga786/hexapod-cpg},
  license = {MIT}
}
```

---

## License

The software in this repository, including the `hexapod_cpg` package, is
released under the [MIT License](LICENSE). The chassis CAD and the sketches
in `firmware/legacy_ps2/` and `firmware/ps2_test/` derive from Mark W's
open-source hexapod ([projectsofdan.com](https://www.projectsofdan.com/));
see the note at the end of [`LICENSE`](LICENSE).

<div align="center">
<sub>Field photographs courtesy of Fort Lewis College, used with permission.</sub>
</div>
