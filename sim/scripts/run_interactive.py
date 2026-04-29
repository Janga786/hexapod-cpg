"""
Interactive CPG dashboard
=========================

Launches the full four-panel matplotlib dashboard described in
:mod:`hexapod_cpg.viz`.  Use the keyboard controls listed in the
status panel (or in the README) to drive the robot.

Run with::

    python sim/scripts/run_interactive.py
"""

from __future__ import annotations

import sys

from hexapod_cpg import CPGViz, pick_backend

BANNER = r"""
==============================================================
  Hexapod CPG  --  Six Coupled Kuramoto Oscillators
==============================================================

  Each leg = one phase oscillator  phi_i in [0, 2*pi)

  Coupling equation:
    dphi_i/dt = omega + sum_j K_ij * sin(phi_j - phi_i - theta_ij)

  Gait emerges automatically from omega (drive frequency):
    omega ~ 1.0  -->  Wave gait     (5 legs down, one lifts)
    omega ~ 3.0  -->  Ripple gait   (4 legs down)
    omega ~ 5.0  -->  Tripod gait   (3 legs down, classic fast gait)

  Watch the PHASE WHEEL: each dot is one leg's phase on a circle.
  Watch the GAIT DIAGRAM: stance/swing history rolls left to right.

  Pattern coherence  (1 = perfect lock, 0 = scattered phases)
  is shown as the radius of the green ring on the wheel and as
  "r" in the status panel.

  Keyboard:
    W/S       walk forward / backward
    A/D       rotate left / right
    Up/Down   change omega (watch gait shift on the diagram)
    P         kick a random oscillator (watch re-synchronisation)
    R         randomise all phases (watch CPG converge from scratch)
    Space     stop / start
    Q         quit
"""


def main() -> int:
    print(BANNER)

    backend = pick_backend()
    if backend == "Agg":
        print("ERROR: no interactive matplotlib backend available.")
        print("       Try `pip install pyqt5` or use one of the headless")
        print("       scripts under sim/scripts/render_*.py instead.")
        return 1

    print(f"  Backend: {backend}\n")
    CPGViz().show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
