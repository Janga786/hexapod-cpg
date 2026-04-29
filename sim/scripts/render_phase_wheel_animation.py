"""
Render the phase-wheel converging from random initial phases to a
locked tripod gait, as an animated GIF.

Demonstrates the *emergence* aspect of the CPG: at t=0 the six dots
are scattered around the unit circle; by t≈3 s they have collapsed
into two anti-phase clusters (the tripod attractor).

Output
------
- ``animations/phase_wheel_lockin.gif``
"""

from __future__ import annotations

import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from _paths import ANIM_DIR
from hexapod_cpg import LEG_COLS, LEGS, CPGNetwork
from hexapod_cpg.colors import BG, GRID_COL, PANEL_BG, gait_colour
from hexapod_cpg.gait import duty_factor
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle


def main() -> None:
    cpg = CPGNetwork(omega=5.0, rng=np.random.default_rng(42))
    cpg.set_walking(0.0)

    fps = 24
    duration = 5.0
    n_frames = int(duration * fps)
    dt = 1.0 / fps

    fig, ax = plt.subplots(figsize=(7, 7), facecolor=BG)
    ax.set_facecolor(PANEL_BG)
    for s in ax.spines.values():
        s.set_edgecolor(GRID_COL)
    ax.set_aspect("equal")
    ax.set_xlim(-1.65, 1.65)
    ax.set_ylim(-1.65, 1.65)
    ax.set_xticks([])
    ax.set_yticks([])

    # Outer circle
    ta = np.linspace(0, 2 * math.pi, 300)
    ax.plot(np.cos(ta), np.sin(ta), color="#333355", lw=1.5)

    duty_rad = 2 * math.pi * duty_factor(cpg.omega)
    sect = np.linspace(0, duty_rad, 120)
    ax.fill(
        np.concatenate([[0], np.cos(sect), [0]]),
        np.concatenate([[0], np.sin(sect), [0]]),
        color="#0d2a0d", alpha=0.6, zorder=1,
    )
    sect2 = np.linspace(duty_rad, 2 * math.pi, 120)
    ax.fill(
        np.concatenate([[0], np.cos(sect2), [0]]),
        np.concatenate([[0], np.sin(sect2), [0]]),
        color="#1a0a2a", alpha=0.5, zorder=1,
    )

    coh_ring = Circle((0, 0), 0.0, fill=False, lw=3,
                      edgecolor=gait_colour(cpg.omega), zorder=3)
    ax.add_patch(coh_ring)

    dots: dict[str, object] = {}
    lbls: dict[str, object] = {}
    for leg in LEGS:
        dots[leg], = ax.plot([], [], "o", color=LEG_COLS[leg],
                             markersize=14, zorder=5)
        lbls[leg] = ax.text(0, 0, leg, color=LEG_COLS[leg],
                            fontsize=8, fontweight="bold",
                            ha="center", va="center", zorder=6)

    ax.set_title(
        "Phase wheel — random init  ➜  tripod attractor",
        color="#ccccee", fontsize=13, fontweight="bold", pad=10,
    )
    coh_txt = ax.text(0, -1.50, "r = 0.00", color="#aaffaa",
                      fontsize=11, ha="center")
    time_txt = ax.text(0, 1.50, "t = 0.0 s", color="#ffcc44",
                       fontsize=10, ha="center")

    for angle, label in [(0, "0"), (math.pi / 2, "π/2"),
                         (math.pi, "π"), (3 * math.pi / 2, "3π/2")]:
        ax.text(1.58 * math.cos(angle), 1.58 * math.sin(angle),
                label, color="#444466", fontsize=7,
                ha="center", va="center")

    def update(k: int):
        cpg.step(dt)
        for idx, leg in enumerate(LEGS):
            phi = cpg.phi[idx]
            x = 1.18 * math.cos(phi)
            y = 1.18 * math.sin(phi)
            dots[leg].set_data([x], [y])
            lbls[leg].set_position((1.40 * math.cos(phi),
                                    1.40 * math.sin(phi)))
        coh = cpg.phase_coherence()
        coh_ring.set_radius(coh * 0.88)
        coh_ring.set_edgecolor(gait_colour(cpg.omega))
        coh_txt.set_text(f"r = {coh:.2f}")
        time_txt.set_text(f"t = {(k * dt):.1f} s")
        return ()

    anim = FuncAnimation(fig, update, frames=n_frames,
                         interval=1000 / fps, blit=False)

    out = ANIM_DIR / "phase_wheel_lockin.gif"
    anim.save(str(out), writer=PillowWriter(fps=fps))
    plt.close(fig)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
