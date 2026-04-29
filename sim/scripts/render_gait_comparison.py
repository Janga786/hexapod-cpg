"""
Render side-by-side gait diagrams for wave, ripple and tripod.

Drives a fresh :class:`CPGNetwork` at three different omegas, lets the
network lock in for a few cycles, then dumps the resulting stance/swing
pattern as a three-panel figure.

Output
------
- ``figures/gait_comparison.png``
"""

from __future__ import annotations

import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from _paths import FIG_DIR
from hexapod_cpg import LEG_COLS, LEGS, CPGNetwork
from hexapod_cpg.colors import BG, GRID_COL, PANEL_BG
from hexapod_cpg.gait import duty_factor, gait_label_from_omega

GAITS = [
    ("Wave",   1.0,  10.0),
    ("Ripple", 3.0,  10.0),
    ("Tripod", 5.0,  10.0),
]


def collect_history(omega: float, duration_s: float) -> list[tuple[float, np.ndarray]]:
    """Run the network at ``omega`` for ``duration_s`` and return its history."""
    cpg = CPGNetwork(omega=omega, rng=np.random.default_rng(7))
    cpg.set_walking(0.0)

    # warm-up so we are looking at a locked-in pattern, not the
    # convergence transient.
    for _ in range(int(2.0 / 0.02)):
        cpg.step(0.02)
    cpg.history.clear()

    n_steps = int(duration_s / 0.02)
    for _ in range(n_steps):
        cpg.step(0.02)
    return list(cpg.history)


def main() -> None:
    fig, axes = plt.subplots(
        3, 1,
        figsize=(11, 7.5),
        sharex=True,
        facecolor=BG,
    )

    for ax_idx, (label, omega, dur) in enumerate(GAITS):
        ax = axes[ax_idx]
        ax.set_facecolor(PANEL_BG)
        for s in ax.spines.values():
            s.set_edgecolor(GRID_COL)

        history = collect_history(omega, dur)
        duty_rad = 2 * math.pi * duty_factor(omega)

        for idx, leg in enumerate(LEGS):
            xs, ys, cs = [], [], []
            for t, phases in history:
                xs.append(t - history[0][0])
                ys.append(idx)
                cs.append(LEG_COLS[leg] if phases[idx] >= duty_rad else "#1a1a3a")
            ax.scatter(xs, ys, c=cs, marker="|", linewidths=2.6, s=18)
            ax.axhline(idx, color="#12122a", lw=8, zorder=0)

        ax.set_yticks(range(6))
        ax.set_yticklabels(LEGS, color="#aabbff", fontsize=9)
        ax.tick_params(axis="x", colors="#666688", labelsize=8)
        ax.tick_params(axis="y", length=0)
        ax.set_title(
            f"{label}  (ω = {omega:.1f} rad/s,  duty = {duty_factor(omega):.2f},  "
            f"label = {gait_label_from_omega(omega)})",
            color="#aabbff", fontsize=10, fontweight="bold", pad=4,
        )

    axes[-1].set_xlabel("Time (s)", color="#8888aa", fontsize=9)
    fig.suptitle(
        "Gait diagrams — emergent from CPG natural-frequency drive only",
        color="#ccccee", fontsize=12, fontweight="bold", y=0.98,
    )
    plt.tight_layout(rect=[0, 0, 1, 0.96])

    out = FIG_DIR / "gait_comparison.png"
    plt.savefig(out, dpi=150, facecolor=BG)
    plt.close()
    print(f"saved {out}")


if __name__ == "__main__":
    main()
