"""
Render an inverse-kinematics workspace map for a single leg.

Sweeps the foot through the (x, z) plane on a fine grid at the leg's
neutral y, evaluates :func:`hexapod_cpg.geometry.ik`, and plots a
heat-map of which targets are reachable + within servo limits.
Overlays the neutral foot point and the swing/stance arc.

Output
------
- ``figures/leg_workspace.png``
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from _paths import FIG_DIR
from hexapod_cpg import (
    DEFAULT_Z,
    MAX_REACH,
    MOUNTS,
    NEUTRAL,
    STEP_HEIGHT,
    Vec3,
    ik,
)
from hexapod_cpg.colors import BG, GRID_COL, PANEL_BG


def main() -> None:
    leg = "RM"
    mx, my = MOUNTS[leg]
    nx, ny = NEUTRAL[leg]

    # x-z grid in body frame, at the leg's neutral y.
    x_lo = mx - 50
    x_hi = mx + int(MAX_REACH) + 30
    z_lo = -220
    z_hi = +60

    xs = np.linspace(x_lo, x_hi, 220)
    zs = np.linspace(z_lo, z_hi, 220)
    XX, ZZ = np.meshgrid(xs, zs)

    valid = np.zeros_like(XX, dtype=bool)
    for i in range(XX.shape[0]):
        for j in range(XX.shape[1]):
            ang = ik(Vec3(XX[i, j], ny, ZZ[i, j]), mx, my)
            valid[i, j] = ang.valid

    fig, ax = plt.subplots(figsize=(9.5, 6.8), facecolor=BG)
    ax.set_facecolor(PANEL_BG)
    for s in ax.spines.values():
        s.set_edgecolor(GRID_COL)

    ax.imshow(
        valid.astype(float),
        extent=(x_lo, x_hi, z_lo, z_hi),
        origin="lower",
        aspect="equal",
        cmap="cividis",
        alpha=0.85,
    )

    # Neutral foot
    ax.scatter([nx], [DEFAULT_Z], color="#ffaa44", s=100,
               marker="*", zorder=5, label="neutral foot")

    # Stance/swing envelope (illustrative)
    stride = 65.0
    ax.plot([nx - stride / 2, nx + stride / 2],
            [DEFAULT_Z, DEFAULT_Z],
            color="#44ff88", lw=2.2, label="stance segment")

    swing_x = np.linspace(nx + stride / 2, nx - stride / 2, 50)
    swing_t = np.linspace(0, 1, 50)
    swing_z = DEFAULT_Z + STEP_HEIGHT * np.sin(np.pi * swing_t)
    ax.plot(swing_x, swing_z, color="#aa66ff", lw=2.2, label="swing arc")

    # Mount marker
    ax.scatter([mx], [0], color="#cccccc", s=80, marker="s",
               zorder=5, label="coxa mount")

    ax.set_xlabel("x  [mm]  (forward)", color="#aabbff", fontsize=10)
    ax.set_ylabel("z  [mm]  (vertical)", color="#aabbff", fontsize=10)
    ax.tick_params(colors="#666688", labelsize=8)
    ax.set_title(
        f"Leg {leg} reachable workspace  (y = {ny:.1f} mm)\n"
        "yellow = reachable + within servo limits   ·   black = unreachable",
        color="#ccccee", fontsize=11, fontweight="bold", pad=8,
    )
    ax.legend(loc="upper right", fontsize=9,
              labelcolor="#ddddee", framealpha=0.0)
    ax.set_xlim(x_lo, x_hi)
    ax.set_ylim(z_lo, z_hi)

    plt.tight_layout()
    out = FIG_DIR / "leg_workspace.png"
    plt.savefig(out, dpi=150, facecolor=BG, bbox_inches="tight")
    plt.close()
    print(f"saved {out}")


if __name__ == "__main__":
    main()
