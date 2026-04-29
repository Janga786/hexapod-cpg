"""
Render foot Cartesian trajectories for one cycle of each gait.

For each gait (wave, ripple, tripod) we plot the resulting (x, y, z)
foot path over a single full cycle, which makes the stance/swing split
and the smoothstep swing arc directly visible.

Output
------
- ``figures/foot_trajectories.png``
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

GAITS = [("Wave", 1.0), ("Ripple", 3.0), ("Tripod", 5.0)]


def collect_foot_trajectory(omega: float, n_cycles: float = 1.5) -> dict[str, np.ndarray]:
    """Walk forward at ``omega`` and return per-leg (N,3) foot paths."""
    cpg = CPGNetwork(omega=omega, rng=np.random.default_rng(11))
    cpg.set_walking(0.0)

    # Allow the network to settle.
    for _ in range(int(2.0 / 0.02)):
        cpg.step(0.02)

    cycle_period = 2 * math.pi / omega
    n_steps = int(n_cycles * cycle_period / 0.02)

    paths: dict[str, list[tuple[float, float, float]]] = {leg: [] for leg in LEGS}
    for _ in range(n_steps):
        feet = cpg.step(0.02)
        for leg in LEGS:
            f = feet[leg]
            paths[leg].append((f.x, f.y, f.z))
    return {leg: np.asarray(paths[leg]) for leg in LEGS}


def main() -> None:
    fig = plt.figure(figsize=(15, 5), facecolor=BG)
    for col_idx, (label, omega) in enumerate(GAITS):
        paths = collect_foot_trajectory(omega)

        ax = fig.add_subplot(1, 3, col_idx + 1, projection="3d")
        ax.set_facecolor(PANEL_BG)
        for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
            pane.fill = False
            pane.set_edgecolor(GRID_COL)
        ax.set_title(
            f"{label} gait  (ω = {omega:.1f})",
            color="#ccccee", fontsize=12, fontweight="bold", pad=6,
        )

        for leg in LEGS:
            p = paths[leg]
            ax.plot(p[:, 0], p[:, 1], p[:, 2],
                    color=LEG_COLS[leg], lw=1.5, alpha=0.9)
            ax.scatter(p[0, 0], p[0, 1], p[0, 2],
                       color=LEG_COLS[leg], s=35, marker="o", zorder=5)

        ax.tick_params(colors="#556", labelsize=6)
        ax.set_xlabel("x [mm]", color="#556", fontsize=7, labelpad=2)
        ax.set_ylabel("y [mm]", color="#556", fontsize=7, labelpad=2)
        ax.set_zlabel("z [mm]", color="#556", fontsize=7, labelpad=2)
        ax.view_init(elev=22, azim=-60)
        ax.set_box_aspect([1, 1, 0.5])

    fig.suptitle(
        "Foot trajectories — one cycle per gait, drawn from CPG output",
        color="#ccccee", fontsize=13, fontweight="bold", y=0.98,
    )
    plt.tight_layout(rect=[0, 0, 1, 0.94])
    out = FIG_DIR / "foot_trajectories.png"
    plt.savefig(out, dpi=150, facecolor=BG, bbox_inches="tight")
    plt.close()
    print(f"saved {out}")


if __name__ == "__main__":
    main()
