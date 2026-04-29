"""
Render an animated GIF of the hexapod walking, for each gait.

Produces a 3-D stick-figure animation (one GIF per gait) showing the
body, the six legs colour-coded by chain (coxa = blue, femur = green,
tibia = red), and the foot trail in orange.  Designed to be embedded
directly in the README gallery.

Output
------
- ``animations/walk_wave.gif``
- ``animations/walk_ripple.gif``
- ``animations/walk_tripod.gif``
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from _paths import ANIM_DIR
from hexapod_cpg import (
    BODY_POLY,
    DEFAULT_Z,
    LEG_COLS,
    LEGS,
    MOUNTS,
    CPGNetwork,
    ik,
    joint_pts,
)
from hexapod_cpg.colors import (
    BG,
    COL_BODY,
    COL_COXA,
    COL_FEMUR,
    COL_FOOT_DOWN,
    COL_FOOT_UP,
    COL_JOINT,
    COL_TIBIA,
    GRID_COL,
    PANEL_BG,
    gait_colour,
)
from matplotlib.animation import FuncAnimation, PillowWriter

GAITS = [
    ("wave",   1.0,  6.0, 18),
    ("ripple", 3.0,  6.0, 18),
    ("tripod", 5.0,  6.0, 22),
]


def render_one(label: str, omega: float, duration_s: float, fps: int) -> None:
    cpg = CPGNetwork(omega=omega, rng=np.random.default_rng(13))
    cpg.set_walking(0.0)

    # let it lock in
    for _ in range(int(2.0 / 0.02)):
        cpg.step(0.02)

    n_frames = int(duration_s * fps)
    dt = 1.0 / fps

    # collect frames first so we can render with a consistent FuncAnimation loop
    foot_history: list[dict] = []
    for _ in range(n_frames):
        feet = cpg.step(dt)
        foot_history.append(feet)

    fig = plt.figure(figsize=(8, 7), facecolor=BG)
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor(PANEL_BG)
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.fill = False
        pane.set_edgecolor(GRID_COL)

    ax.set_xlim(-310, 310)
    ax.set_ylim(-310, 310)
    ax.set_zlim(-200, 90)
    ax.set_box_aspect([1, 1, 0.7])
    ax.view_init(elev=24, azim=-55)
    ax.tick_params(colors="#556", labelsize=6)
    ax.set_xlabel("x [mm]", color="#556", fontsize=7)
    ax.set_ylabel("y [mm]", color="#556", fontsize=7)
    ax.set_zlabel("z [mm]", color="#556", fontsize=7)

    ax.set_title(
        f"Hexapod CPG — {label.upper()} gait  (ω = {omega:.1f} rad/s)",
        color=gait_colour(omega), fontsize=12,
        fontweight="bold", pad=8,
    )

    # ground plane
    xs = np.linspace(-310, 310, 2)
    xx, yy = np.meshgrid(xs, xs)
    ax.plot_surface(xx, yy, np.full_like(xx, DEFAULT_Z - 0.5),
                    alpha=0.06, color="#4444aa", zorder=0)

    # body polyline
    body_pts = [MOUNTS[n] for n in BODY_POLY] + [MOUNTS[BODY_POLY[0]]]
    body_line, = ax.plot(
        [p[0] for p in body_pts],
        [p[1] for p in body_pts],
        [0.0] * len(body_pts),
        "-", color=COL_BODY, lw=2.5, zorder=4,
    )

    leg_art: dict[str, dict] = {}
    trace_art: dict[str, object] = {}
    foot_traces: dict[str, list] = {leg: [] for leg in LEGS}
    for leg in LEGS:
        leg_art[leg] = {
            "coxa":  ax.plot([], [], [], "-", color=COL_COXA,  lw=4, zorder=5)[0],
            "femur": ax.plot([], [], [], "-", color=COL_FEMUR, lw=4, zorder=5)[0],
            "tibia": ax.plot([], [], [], "-", color=COL_TIBIA, lw=4, zorder=5)[0],
            "joints": ax.scatter([], [], [], s=22, c=COL_JOINT,  zorder=6, depthshade=False),
            "foot":   ax.scatter([], [], [], s=60, c=COL_FOOT_DOWN, zorder=7, depthshade=False),
        }
        trace_art[leg] = ax.plot([], [], [], "-",
                                 color=LEG_COLS[leg], lw=0.8,
                                 alpha=0.55, zorder=2)[0]

    def update(k: int):
        feet = foot_history[k]
        for leg in LEGS:
            mx, my = MOUNTS[leg]
            ang = ik(feet[leg], mx, my)
            if not ang.valid:
                continue
            mount, ce, fe, ft = joint_pts(ang, mx, my)
            a = leg_art[leg]
            a["coxa"].set_data_3d([mount[0], ce[0]],
                                  [mount[1], ce[1]],
                                  [mount[2], ce[2]])
            a["femur"].set_data_3d([ce[0], fe[0]],
                                   [ce[1], fe[1]],
                                   [ce[2], fe[2]])
            a["tibia"].set_data_3d([fe[0], ft[0]],
                                   [fe[1], ft[1]],
                                   [fe[2], ft[2]])
            jp = np.array([mount, ce, fe])
            a["joints"]._offsets3d = (jp[:, 0], jp[:, 1], jp[:, 2])

            is_up = feet[leg].z > DEFAULT_Z + 5.0
            a["foot"]._offsets3d = ([ft[0]], [ft[1]], [ft[2]])
            a["foot"].set_color(COL_FOOT_UP if is_up else COL_FOOT_DOWN)
            a["foot"].set_sizes([95 if is_up else 60])

            foot_traces[leg].append(ft.copy())
            if len(foot_traces[leg]) > 130:
                foot_traces[leg].pop(0)
            tr = np.asarray(foot_traces[leg])
            trace_art[leg].set_data_3d(tr[:, 0], tr[:, 1], tr[:, 2])
        return ()

    anim = FuncAnimation(fig, update, frames=n_frames,
                         interval=1000 / fps, blit=False)

    out = ANIM_DIR / f"walk_{label}.gif"
    writer = PillowWriter(fps=fps)
    anim.save(str(out), writer=writer)
    plt.close(fig)
    print(f"saved {out}")


def main() -> None:
    for label, omega, duration_s, fps in GAITS:
        render_one(label, omega, duration_s, fps)


if __name__ == "__main__":
    main()
