"""
Render an ω-sweep figure showing duty factor and gait label vs ω.

Sweeps ω across the full operating range [0.5, 6.0] rad/s and plots
the duty factor and the smoothstep blend coefficient α(ω), with the
three gait labels (Wave / Ripple / Tripod) annotated as background
bands.

Output
------
- ``figures/omega_sweep.png``
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from _paths import FIG_DIR
from hexapod_cpg.colors import BG, GRID_COL, PANEL_BG, gait_colour
from hexapod_cpg.gait import duty_factor, gait_label_from_omega, smoothstep_alpha


def main() -> None:
    omegas = np.linspace(0.5, 6.0, 400)
    duty = np.array([duty_factor(o) for o in omegas])
    alpha = np.array([smoothstep_alpha(o) for o in omegas])
    labels = [gait_label_from_omega(o) for o in omegas]

    fig, axes = plt.subplots(2, 1, figsize=(11, 6.5),
                             sharex=True, facecolor=BG)

    for ax in axes:
        ax.set_facecolor(PANEL_BG)
        for s in ax.spines.values():
            s.set_edgecolor(GRID_COL)
        ax.tick_params(colors="#666688", labelsize=8)

    # gait-band background
    last_label = labels[0]
    band_start = omegas[0]
    for o, lbl in zip(omegas, labels, strict=False):
        if lbl != last_label:
            for ax in axes:
                ax.axvspan(band_start, o,
                           color=gait_colour((band_start + o) / 2),
                           alpha=0.10, zorder=0)
            band_start = o
            last_label = lbl
    for ax in axes:
        ax.axvspan(band_start, omegas[-1],
                   color=gait_colour((band_start + omegas[-1]) / 2),
                   alpha=0.10, zorder=0)

    # duty factor
    ax = axes[0]
    ax.plot(omegas, duty, color="#44ccff", lw=2.4)
    ax.set_ylabel("duty factor  β", color="#aabbff", fontsize=10)
    ax.set_ylim(0.45, 0.78)
    ax.set_title(
        "Gait properties as a function of natural-frequency drive ω",
        color="#ccccee", fontsize=12, fontweight="bold", pad=6,
    )

    # smoothstep blend
    ax = axes[1]
    ax.plot(omegas, alpha, color="#ff8844", lw=2.4)
    ax.set_xlabel("ω  [rad/s]", color="#aabbff", fontsize=10)
    ax.set_ylabel("blend α(ω)\n0=wave, 1=tripod", color="#aabbff", fontsize=10)
    ax.set_ylim(-0.05, 1.05)

    # band labels
    for label, x_centre in [("WAVE", 1.0), ("RIPPLE", 3.0), ("TRIPOD", 5.0)]:
        axes[0].annotate(
            label, (x_centre, 0.755),
            ha="center", color=gait_colour(x_centre),
            fontsize=11, fontweight="bold",
        )

    plt.tight_layout()
    out = FIG_DIR / "omega_sweep.png"
    plt.savefig(out, dpi=150, facecolor=BG, bbox_inches="tight")
    plt.close()
    print(f"saved {out}")


if __name__ == "__main__":
    main()
