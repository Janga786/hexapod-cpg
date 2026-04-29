"""
Interactive 3-D visualiser
==========================

A four-panel matplotlib dashboard:

- **Left** (large): a 3-D stick-figure render of the hexapod, with
  per-leg link colours (coxa = blue, femur = green, tibia = red), foot
  markers (orange = stance, purple = swing) and translucent foot-trail
  traces.
- **Top right**: the **phase wheel** — six dots on a unit circle,
  colour-coded by leg, with a stance/swing sector overlay and a
  shrinking ring whose radius is the gait-coherence metric.
- **Middle right**: a rolling **gait diagram** showing stance/swing
  history for the last :data:`HISTORY_SECS` seconds.
- **Bottom right**: a status panel with mode, ω, gait label, the
  coherence number, and per-leg joint angles.

Everything is driven from one ``FuncAnimation`` callback that, each
frame, advances the CPG by ``dt`` and pushes the resulting foot
positions through inverse kinematics into the 3-D scene.

Keyboard
--------

============ =====================================
``W``        Walk forward
``S``        Walk backward
``A`` / ``D`` Turn left / right
``Space``    Stop / start
``↑`` / ``↓``  Increase / decrease ω (gait shifts)
``P``        Perturb a random oscillator
``R``        Randomise all phases
``Q``        Quit
============ =====================================
"""

from __future__ import annotations

import contextlib
import math
import sys
import time

import matplotlib
import matplotlib.gridspec as gridspec
import numpy as np

from .colors import (
    BG,
    COL_BODY,
    COL_COXA,
    COL_FEMUR,
    COL_FOOT_DOWN,
    COL_FOOT_UP,
    COL_JOINT,
    COL_TIBIA,
    GRID_COL,
    LEG_COLS,
    PANEL_BG,
    gait_colour,
)
from .cpg import CPGNetwork
from .gait import duty_factor
from .geometry import (
    BODY_POLY,
    DEFAULT_Z,
    LEGS,
    MOUNTS,
    N,
    ik,
    joint_pts,
)


def pick_backend() -> str:
    """
    Try interactive matplotlib backends in order of preference and
    return the name of the first one that can open a figure window;
    fall back to ``Agg`` (non-interactive) if none work.

    Centralises the ugly try/except dance so that scripts and notebooks
    can branch on the result.
    """
    candidates = [
        "macosx",
        "Qt5Agg",
        "Qt6Agg",
        "GTK4Agg",
        "GTK3Agg",
        "wxAgg",
        "TkAgg",
        "Agg",
    ]
    for name in candidates:
        try:
            matplotlib.use(name, force=True)
            import matplotlib.pyplot as _p
            _p.figure()
            _p.close("all")
            return name
        except Exception:
            continue
    return "Agg"


class CPGViz:
    """
    The dashboard.  Construct, then call :meth:`show`.

    Parameters
    ----------
    cpg
        Optional :class:`hexapod_cpg.cpg.CPGNetwork`.  A fresh one is
        constructed if omitted.
    """

    HISTORY_SECS: float = 4.0

    def __init__(self, cpg: CPGNetwork | None = None) -> None:
        import matplotlib.pyplot as plt

        self.fig = plt.figure(figsize=(16, 9), facecolor=BG)
        with contextlib.suppress(Exception):
            self.fig.canvas.manager.set_window_title("Hexapod CPG Simulation")

        gs = gridspec.GridSpec(
            3, 2,
            figure=self.fig,
            width_ratios=[1.55, 1.0],
            height_ratios=[1.1, 1.0, 0.85],
            hspace=0.40, wspace=0.07,
            left=0.01, right=0.97, top=0.94, bottom=0.05,
        )
        self.ax3d = self.fig.add_subplot(gs[:, 0], projection="3d")
        self.ax_whl = self.fig.add_subplot(gs[0, 1])
        self.ax_gdt = self.fig.add_subplot(gs[1, 1])
        self.ax_inf = self.fig.add_subplot(gs[2, 1])

        self._style_all()
        self._init_3d()
        self._init_wheel()
        self._init_gait_diagram()
        self._init_info()

        self.cpg: CPGNetwork = cpg if cpg is not None else CPGNetwork()
        self.mode: str = "stand"
        self.traces: dict[str, list[np.ndarray]] = {leg: [] for leg in LEGS}
        self._last_t: float | None = None

        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

        import matplotlib.animation as animation

        self._anim = animation.FuncAnimation(
            self.fig, self._animate,
            interval=33, blit=False, cache_frame_data=False,
        )

    # ------------------------------------------------------------------
    # Style
    # ------------------------------------------------------------------

    def _style_all(self) -> None:
        ax = self.ax3d
        ax.set_facecolor(PANEL_BG)
        for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
            pane.fill = False
            pane.set_edgecolor(GRID_COL)
        ax.tick_params(colors="#556", labelsize=6)
        ax.set_xlabel("X fwd", color="#556", labelpad=2, fontsize=7)
        ax.set_ylabel("Y left", color="#556", labelpad=2, fontsize=7)
        ax.set_zlabel("Z up", color="#556", labelpad=2, fontsize=7)
        ax.set_xlim(-310, 310)
        ax.set_ylim(-310, 310)
        ax.set_zlim(-200, 90)
        ax.view_init(elev=24, azim=-55)

        # ground plane
        lim = 310
        xs = np.linspace(-lim, lim, 2)
        xx, yy = np.meshgrid(xs, xs)
        ax.plot_surface(
            xx, yy, np.full_like(xx, DEFAULT_Z - 0.5),
            alpha=0.06, color="#4444aa", zorder=0,
        )

        for a in (self.ax_whl, self.ax_gdt, self.ax_inf):
            a.set_facecolor(PANEL_BG)
            for s in a.spines.values():
                s.set_edgecolor(GRID_COL)

    # ------------------------------------------------------------------
    # 3-D artists
    # ------------------------------------------------------------------

    def _init_3d(self) -> None:
        ax = self.ax3d
        self.leg_art: dict[str, dict] = {}
        for leg in LEGS:
            self.leg_art[leg] = {
                "coxa":  ax.plot([], [], [], "-", color=COL_COXA,  lw=4,
                                 solid_capstyle="round", zorder=5)[0],
                "femur": ax.plot([], [], [], "-", color=COL_FEMUR, lw=4,
                                 solid_capstyle="round", zorder=5)[0],
                "tibia": ax.plot([], [], [], "-", color=COL_TIBIA, lw=4,
                                 solid_capstyle="round", zorder=5)[0],
                "joints": ax.scatter([], [], [], s=22, c=COL_JOINT,
                                     zorder=6, depthshade=False),
                "foot": ax.scatter([], [], [], s=60, c=COL_FOOT_DOWN,
                                   zorder=7, depthshade=False),
            }
        self.body_line, = ax.plot([], [], [], "-", color=COL_BODY,
                                  lw=2.5, zorder=4)
        self.title_3d = ax.set_title(
            "Hexapod CPG", color="#ccccee",
            fontsize=11, fontweight="bold", pad=6,
        )
        self.trace_art = {
            leg: ax.plot([], [], [], "-", color=LEG_COLS[leg],
                         lw=0.8, alpha=0.5, zorder=2)[0]
            for leg in LEGS
        }

    # ------------------------------------------------------------------
    # Phase wheel
    # ------------------------------------------------------------------

    def _init_wheel(self) -> None:
        from matplotlib.patches import Circle

        ax = self.ax_whl
        ax.set_aspect("equal")
        ax.set_xlim(-1.65, 1.65)
        ax.set_ylim(-1.65, 1.65)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title("Phase Wheel", color="#aabbff",
                     fontsize=9, fontweight="bold", pad=4)

        ta = np.linspace(0, 2 * math.pi, 300)
        ax.plot(np.cos(ta), np.sin(ta), "-", color="#333355", lw=1.5)

        self._stance_patch = ax.fill([], [], color="#0d2a0d",
                                     alpha=0.6, zorder=1)[0]
        self._swing_patch = ax.fill([], [], color="#1a0a2a",
                                    alpha=0.5, zorder=1)[0]

        self._whl_dots: dict[str, object] = {}
        self._whl_lbls: dict[str, object] = {}
        for leg in LEGS:
            dot, = ax.plot([], [], "o", color=LEG_COLS[leg],
                           markersize=11, zorder=5)
            lbl = ax.text(0, 0, leg, color=LEG_COLS[leg],
                          fontsize=6.5, fontweight="bold",
                          ha="center", va="center", zorder=6)
            self._whl_dots[leg] = dot
            self._whl_lbls[leg] = lbl

        self._coh_ring = Circle((0, 0), 0.0, fill=False, lw=3,
                                zorder=3, edgecolor="#44ff88")
        ax.add_patch(self._coh_ring)

        self._coh_txt = ax.text(0, -1.50, "r = 0.00",
                                color="#aaffaa", fontsize=8, ha="center")
        self._gait_txt = ax.text(0, 1.48, "STAND", color="#ffcc44",
                                 fontsize=9, ha="center", fontweight="bold")

        for angle, label in [
            (0, "0"), (math.pi / 2, "pi/2"),
            (math.pi, "pi"), (3 * math.pi / 2, "3pi/2"),
        ]:
            ax.text(1.58 * math.cos(angle), 1.58 * math.sin(angle),
                    label, color="#444466", fontsize=6,
                    ha="center", va="center")

    def _update_wheel(self) -> None:
        cpg = self.cpg
        duty_rad = 2 * math.pi * duty_factor(cpg.omega)

        ta = np.linspace(0, duty_rad, 120)
        sx = np.concatenate([[0], np.cos(ta), [0]])
        sy = np.concatenate([[0], np.sin(ta), [0]])
        self._stance_patch.set_xy(np.column_stack([sx, sy]))

        tb = np.linspace(duty_rad, 2 * math.pi, 120)
        swx = np.concatenate([[0], np.cos(tb), [0]])
        swy = np.concatenate([[0], np.sin(tb), [0]])
        self._swing_patch.set_xy(np.column_stack([swx, swy]))

        r_dot = 1.18
        for idx, leg in enumerate(LEGS):
            phi = cpg.phi[idx]
            x = r_dot * math.cos(phi)
            y = r_dot * math.sin(phi)
            self._whl_dots[leg].set_data([x], [y])
            lx = 1.40 * math.cos(phi)
            ly = 1.40 * math.sin(phi)
            self._whl_lbls[leg].set_position((lx, ly))

        coh = cpg.phase_coherence()
        self._coh_ring.set_radius(coh * 0.88)
        self._coh_ring.set_edgecolor(gait_colour(cpg.omega))
        self._coh_txt.set_text(f"r = {coh:.2f}")

        gname = cpg.gait_name() if cpg.moving else "STAND"
        self._gait_txt.set_text(gname)
        self._gait_txt.set_color(
            gait_colour(cpg.omega) if cpg.moving else "#888888"
        )

    # ------------------------------------------------------------------
    # Gait diagram
    # ------------------------------------------------------------------

    def _init_gait_diagram(self) -> None:
        ax = self.ax_gdt
        ax.set_facecolor("#080812")
        ax.set_xlim(0, self.HISTORY_SECS)
        ax.set_ylim(-0.5, N - 0.5)
        ax.set_yticks(range(N))
        ax.set_yticklabels(LEGS, color="#aabbff", fontsize=8)
        ax.set_xlabel("Time (s)", color="#8888aa",
                      fontsize=7, labelpad=2)
        ax.set_title("Gait Diagram  (bright=swing, dark=stance)",
                     color="#aabbff", fontsize=8,
                     fontweight="bold", pad=3)
        ax.tick_params(axis="x", colors="#666688", labelsize=7)
        ax.tick_params(axis="y", length=0)

        for idx in range(N):
            ax.axhline(idx, color="#12122a", lw=10, zorder=1)

        self._gdt_scats: dict[str, object] = {}
        for leg in LEGS:
            scat = ax.scatter([], [], s=14, marker="|",
                              linewidths=2.5, zorder=3, c=LEG_COLS[leg])
            self._gdt_scats[leg] = scat

    def _update_gait_diagram(self) -> None:
        cpg = self.cpg
        hist = list(cpg.history)
        if len(hist) < 2:
            return
        t_now = hist[-1][0]
        t_min = t_now - self.HISTORY_SECS
        duty_rad = 2 * math.pi * duty_factor(cpg.omega)

        for idx, leg in enumerate(LEGS):
            xs, ys, cs = [], [], []
            for t, phases in hist:
                if t < t_min:
                    continue
                tx = t - t_min
                phi = phases[idx]
                xs.append(tx)
                ys.append(idx)
                cs.append(LEG_COLS[leg] if phi >= duty_rad else "#1a1a3a")

            if xs:
                self._gdt_scats[leg].set_offsets(np.column_stack([xs, ys]))
                self._gdt_scats[leg].set_color(cs)
            else:
                self._gdt_scats[leg].set_offsets(np.zeros((0, 2)))

    # ------------------------------------------------------------------
    # Info panel
    # ------------------------------------------------------------------

    def _init_info(self) -> None:
        ax = self.ax_inf
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.set_xticks([])
        ax.set_yticks([])

        def t(x, y, s, **kw):
            return ax.text(x, y, s, transform=ax.transAxes,
                           fontfamily="monospace", **kw)

        t(0.5, 0.97, "CPG STATUS", ha="center", va="top",
          color="#aabbff", fontsize=9, fontweight="bold")

        self.txt_mode = t(0.04, 0.86, "Mode:  STAND",
                          color="#66ffaa", fontsize=8)
        self.txt_omega = t(0.04, 0.76, "omega: 2.00 rad/s",
                           color="#ffcc44", fontsize=8)
        self.txt_gait = t(0.04, 0.66, "Gait:  ---",
                          color="#44ccff", fontsize=8)
        self.txt_coh = t(0.04, 0.56, "r:     0.00",
                         color="#ff9944", fontsize=8)

        t(0.04, 0.46, "-" * 32, color="#222244", fontsize=7)
        t(0.04, 0.40, "Leg  Coxa  Femur  Tibia",
          color="#555577", fontsize=7)

        y = 0.33
        self.txt_ang: dict[str, object] = {}
        for leg in LEGS:
            self.txt_ang[leg] = t(
                0.04, y, f"{leg}   ---   ---   ---",
                color="#888899", fontsize=7,
            )
            y -= 0.054

        t(0.04, -0.03,
          "W/S: fwd/bwd   A/D: rotate\n"
          "Up/Dn: omega -> gait changes\n"
          "P: perturb leg  R: randomise\n"
          "SPACE: stop/go   Q: quit",
          color="#333355", fontsize=7, va="bottom")

    def _update_info(self) -> None:
        cpg = self.cpg
        labels = {
            "stand": "STAND",
            "walk_fwd": "WALK FWD",
            "walk_bwd": "WALK BWD",
            "rot_l": "ROTATE LEFT",
            "rot_r": "ROTATE RIGHT",
        }
        self.txt_mode.set_text(f"Mode:  {labels.get(self.mode, '---')}")
        self.txt_omega.set_text(f"omega: {cpg.omega:.2f} rad/s")
        self.txt_gait.set_text(f"Gait:  {cpg.gait_name()}")
        self.txt_coh.set_text(f"r:     {cpg.phase_coherence():.2f}")
        self.txt_gait.set_color(gait_colour(cpg.omega))
        self.title_3d.set_text(
            f"Hexapod CPG  |  {cpg.gait_name()}  |  "
            f"omega={cpg.omega:.1f}  |  r={cpg.phase_coherence():.2f}"
        )
        self.title_3d.set_color(gait_colour(cpg.omega))

    # ------------------------------------------------------------------
    # Per-frame rendering
    # ------------------------------------------------------------------

    def _render_leg(self, leg, foot) -> None:
        mx, my = MOUNTS[leg]
        angles = ik(foot, mx, my)
        if not angles.valid:
            return
        mount, ce, fe, ft = joint_pts(angles, mx, my)
        a = self.leg_art[leg]

        a["coxa"].set_data_3d(
            [mount[0], ce[0]], [mount[1], ce[1]], [mount[2], ce[2]]
        )
        a["femur"].set_data_3d(
            [ce[0], fe[0]], [ce[1], fe[1]], [ce[2], fe[2]]
        )
        a["tibia"].set_data_3d(
            [fe[0], ft[0]], [fe[1], ft[1]], [fe[2], ft[2]]
        )

        jp = np.array([mount, ce, fe])
        a["joints"]._offsets3d = (jp[:, 0], jp[:, 1], jp[:, 2])

        is_up = foot.z > DEFAULT_Z + 5.0
        a["foot"]._offsets3d = ([ft[0]], [ft[1]], [ft[2]])
        a["foot"].set_color(COL_FOOT_UP if is_up else COL_FOOT_DOWN)
        a["foot"].set_sizes([95 if is_up else 60])

        self.txt_ang[leg].set_text(
            f"{leg}  {angles.coxa:+5.1f} {angles.femur:+5.1f} {angles.tibia:+5.1f}"
        )

        self.traces[leg].append(ft.copy())
        if len(self.traces[leg]) > 130:
            self.traces[leg].pop(0)
        tr = np.array(self.traces[leg])
        self.trace_art[leg].set_data_3d(tr[:, 0], tr[:, 1], tr[:, 2])

    def _render_body(self) -> None:
        pts = [MOUNTS[n] for n in BODY_POLY] + [MOUNTS[BODY_POLY[0]]]
        self.body_line.set_data_3d(
            [p[0] for p in pts],
            [p[1] for p in pts],
            [0.0] * len(pts),
        )

    def _animate(self, frame: int) -> None:
        now = time.monotonic()
        dt = min(now - self._last_t, 0.08) if self._last_t else 0.033
        self._last_t = now

        feet = self.cpg.step(dt)
        for leg in LEGS:
            self._render_leg(leg, feet[leg])
        self._render_body()
        self._update_wheel()
        self._update_gait_diagram()
        self._update_info()

    # ------------------------------------------------------------------
    # Keyboard
    # ------------------------------------------------------------------

    def _on_key(self, ev) -> None:
        import matplotlib.pyplot as plt

        k = ev.key
        if k == "q":
            plt.close("all")
            sys.exit(0)
        elif k == " ":
            if self.cpg.moving:
                self.cpg.stop()
                self.mode = "stand"
            else:
                self._apply_mode()
        elif k == "w":
            self.mode = "walk_fwd"
            self._apply_mode()
        elif k == "s":
            self.mode = "walk_bwd"
            self._apply_mode()
        elif k == "a":
            self.mode = "rot_l"
            self._apply_mode()
        elif k == "d":
            self.mode = "rot_r"
            self._apply_mode()
        elif k == "up":
            self.cpg.set_speed(self.cpg.omega + 0.3)
            if not self.cpg.moving and self.mode != "stand":
                self._apply_mode()
        elif k == "down":
            self.cpg.set_speed(self.cpg.omega - 0.3)
        elif k == "p":
            idx = int(np.random.randint(0, N))
            self.cpg.perturb(idx, math.pi * float(np.random.uniform(0.7, 1.3)))
            print(f"  >> Perturbed {LEGS[idx]} — watch re-synchronisation")
        elif k == "r":
            self.cpg.randomise_phases()
            print("  >> Phases randomised — watch CPG converge to gait")

    def _apply_mode(self) -> None:
        if self.mode == "walk_fwd":
            self.cpg.set_walking(0.0)
        elif self.mode == "walk_bwd":
            self.cpg.set_walking(180.0)
        elif self.mode == "rot_l":
            self.cpg.set_turning(+1.0)
        elif self.mode == "rot_r":
            self.cpg.set_turning(-1.0)

    def show(self) -> None:
        import matplotlib.pyplot as plt
        plt.show()
