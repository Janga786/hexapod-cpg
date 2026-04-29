"""
Render the CPG coupling graph as an annotated network diagram.

Shows the six legs at their physical body-frame mount positions, with
ipsilateral / contralateral / diagonal coupling edges colour-coded.
The figure is the visual companion to the equations in the README.

Output
------
- ``figures/coupling_graph.png``
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from _paths import FIG_DIR
from hexapod_cpg import LEG_COLS, LEGS, MOUNTS
from hexapod_cpg.colors import BG, GRID_COL, PANEL_BG
from hexapod_cpg.cpg import CPGNetwork

# Edge classification (must agree with CPGNetwork.EDGES).
IPSILATERAL = {(0, 1), (1, 2), (3, 4), (4, 5)}
CONTRALATERAL = {(0, 3), (1, 4), (2, 5)}
DIAGONAL = {(0, 4), (2, 4), (3, 1), (5, 1)}

EDGE_COL = {
    "ipsilateral":   "#4fc3f7",
    "contralateral": "#ff8844",
    "diagonal":      "#aa66ff",
}


def classify(edge: tuple[int, int]) -> str:
    if edge in IPSILATERAL:
        return "ipsilateral"
    if edge in CONTRALATERAL:
        return "contralateral"
    return "diagonal"


def main() -> None:
    fig, ax = plt.subplots(figsize=(9, 8), facecolor=BG)
    ax.set_facecolor(PANEL_BG)
    for s in ax.spines.values():
        s.set_edgecolor(GRID_COL)
    ax.set_aspect("equal")

    # Body polygon
    poly_order = ["LF", "RF", "RM", "RB", "LB", "LM", "LF"]
    xs = [MOUNTS[l][0] for l in poly_order]
    ys = [MOUNTS[l][1] for l in poly_order]
    ax.plot(xs, ys, "-", color="#444466", lw=2.5, alpha=0.7)

    # Edges
    seen = set()
    for i, j in CPGNetwork.EDGES:
        key = (min(i, j), max(i, j))
        if key in seen:
            continue
        seen.add(key)
        kind = classify((i, j) if (i, j) in (IPSILATERAL | CONTRALATERAL | DIAGONAL)
                        else (j, i))
        x0, y0 = MOUNTS[LEGS[i]]
        x1, y1 = MOUNTS[LEGS[j]]
        ax.plot([x0, x1], [y0, y1],
                color=EDGE_COL[kind], lw=2.4, alpha=0.85, zorder=2)

    # Nodes
    for leg in LEGS:
        x, y = MOUNTS[leg]
        ax.scatter([x], [y], color=LEG_COLS[leg], s=520,
                   edgecolors="white", lw=1.8, zorder=4)
        ax.text(x, y, leg, color="white", fontsize=10,
                fontweight="bold", ha="center", va="center", zorder=5)

    # Legend
    handles = [
        plt.Line2D([], [], color=EDGE_COL["ipsilateral"],   lw=2.4,
                   label="ipsilateral (same side, adjacent)"),
        plt.Line2D([], [], color=EDGE_COL["contralateral"], lw=2.4,
                   label="contralateral (across body, paired)"),
        plt.Line2D([], [], color=EDGE_COL["diagonal"],      lw=2.4,
                   label="diagonal (cross-body, offset)"),
    ]
    ax.legend(handles=handles, loc="lower center",
              ncols=1, fontsize=9, frameon=False,
              labelcolor="#ddddee", bbox_to_anchor=(0.5, -0.15))

    ax.set_xlim(-110, 110)
    ax.set_ylim(-110, 110)
    ax.set_xlabel("x  [mm]  (forward)", color="#aabbff")
    ax.set_ylabel("y  [mm]  (left)",    color="#aabbff")
    ax.tick_params(colors="#666688", labelsize=8)
    ax.set_title(
        "CPG Coupling Graph — viewed top-down on the body",
        color="#ccccee", fontsize=12, fontweight="bold", pad=8,
    )

    plt.tight_layout()
    out = FIG_DIR / "coupling_graph.png"
    plt.savefig(out, dpi=150, facecolor=BG, bbox_inches="tight")
    plt.close()
    print(f"saved {out}")


if __name__ == "__main__":
    main()
