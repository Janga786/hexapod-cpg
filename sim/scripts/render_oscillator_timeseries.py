"""
Render the six oscillator phase timeseries plus a coherence overlay.

The figure has two stacked panels:

1. ``φᵢ(t) mod 2π`` for every leg, colour-coded.
2. The pattern-coherence metric ``r_pattern(t)`` (and the classical
   Kuramoto order ``r``) over the same interval.

The simulation is initialised from random phases and run for a few
seconds at a fixed ω; the figure then makes the convergence visually
obvious.

Output
------
- ``figures/oscillator_timeseries.png``
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


def main() -> None:
    rng = np.random.default_rng(3)
    cpg = CPGNetwork(omega=4.5, rng=rng)
    cpg.set_walking(0.0)

    dt = 0.02
    n_steps = int(6.0 / dt)
    t = np.arange(n_steps) * dt

    phases = np.zeros((n_steps, 6))
    coh_pattern = np.zeros(n_steps)
    coh_kuramoto = np.zeros(n_steps)

    for k in range(n_steps):
        cpg.step(dt)
        phases[k] = cpg.phi.copy()
        coh_pattern[k] = cpg.phase_coherence()
        coh_kuramoto[k] = cpg.kuramoto_order()

    fig, axes = plt.subplots(
        2, 1, figsize=(12, 7),
        sharex=True, facecolor=BG,
        gridspec_kw={"height_ratios": [2.2, 1]},
    )

    # ── Phase trajectories ──
    ax = axes[0]
    ax.set_facecolor(PANEL_BG)
    for s in ax.spines.values():
        s.set_edgecolor(GRID_COL)
    for idx, leg in enumerate(LEGS):
        ax.plot(t, phases[:, idx], color=LEG_COLS[leg], lw=1.4,
                label=leg, alpha=0.95)
    ax.set_ylabel("phase ϕᵢ(t)  [rad]", color="#aabbff", fontsize=10)
    ax.set_ylim(-0.2, 2 * math.pi + 0.2)
    ax.set_yticks([0, math.pi / 2, math.pi, 3 * math.pi / 2, 2 * math.pi])
    ax.set_yticklabels(["0", "π/2", "π", "3π/2", "2π"],
                       color="#888899", fontsize=8)
    ax.tick_params(axis="x", colors="#666688", labelsize=8)
    ax.legend(loc="upper right", ncols=6, fontsize=8,
              labelcolor="#ddddee", framealpha=0.0)
    ax.set_title(
        "Random initial phases lock to a tripod gait under CPG dynamics",
        color="#ccccee", fontsize=11, fontweight="bold", pad=6,
    )

    # ── Coherence ──
    ax = axes[1]
    ax.set_facecolor(PANEL_BG)
    for s in ax.spines.values():
        s.set_edgecolor(GRID_COL)
    ax.plot(t, coh_pattern, color="#44ff88", lw=2.0,
            label=r"pattern coherence  $r_{\mathrm{pat}}$")
    ax.plot(t, coh_kuramoto, color="#ff8844", lw=1.2, ls="--",
            label=r"Kuramoto order  $r$")
    ax.axhline(1.0, color="#444466", lw=0.6)
    ax.set_ylim(-0.05, 1.05)
    ax.set_ylabel("coherence", color="#aabbff", fontsize=10)
    ax.set_xlabel("Time (s)", color="#8888aa", fontsize=9)
    ax.tick_params(axis="both", colors="#666688", labelsize=8)
    ax.legend(loc="lower right", fontsize=9,
              labelcolor="#ddddee", framealpha=0.0)

    plt.tight_layout()
    out = FIG_DIR / "oscillator_timeseries.png"
    plt.savefig(out, dpi=150, facecolor=BG)
    plt.close()
    print(f"saved {out}")


if __name__ == "__main__":
    main()
