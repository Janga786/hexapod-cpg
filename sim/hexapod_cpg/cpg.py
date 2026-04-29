"""
Central Pattern Generator — six coupled Kuramoto oscillators
============================================================

Each leg is a phase oscillator :math:`\\varphi_i \\in [0, 2\\pi)` and
the six oscillators are coupled by a directed graph that encodes
**ipsilateral** (same-side adjacent: LF-LM, LM-LB, RF-RM, RM-RB),
**contralateral** (across-body: LF-RF, LM-RM, LB-RB) and **diagonal**
(cross-body offset: LF-RM, RF-LM, LB-RM, RB-LM) edges.

Dynamics
--------

.. math::

   \\dot \\varphi_i = \\omega
                  + \\sum_{j} K_{ij} \\sin(\\varphi_j - \\varphi_i - \\theta_{ij}(\\omega))
                  + \\eta_i,

with :math:`\\eta_i \\sim \\mathcal{N}(0, \\sigma)` injecting a small
amount of noise so emergence remains visible from random initial
conditions.

The phase-offset matrix :math:`\\theta_{ij}(\\omega)` smoothly
interpolates between two reference patterns:

- :math:`\\theta^{\\mathrm{wave}}` — even 60° spacing front-to-back
  (LF=0°, LM=60°, LB=120°, RF=180°, RM=240°, RB=300°);
- :math:`\\theta^{\\mathrm{tripod}}` — two anti-phase groups
  (group A: LF, LB, RM at 0°; group B: RF, RB, LM at 180°).

The interpolation parameter is :math:`\\alpha(\\omega) =
\\mathrm{smoothstep}((\\omega-1)/4)`, giving a wave gait at low
:math:`\\omega` and a tripod gait at high :math:`\\omega`, with a
ripple-like pattern in between.

Integration
-----------

A second-order **explicit midpoint (Runge-Kutta-2) scheme** is used.
At a target time-step of 30 ms (33 Hz visual frame rate) it is stable
for the coupling strengths used here (`K_base = 3.5`) and substantially
more accurate than forward Euler at the same cost.

The :meth:`CPGNetwork.step` method is therefore both *the integrator
and the rendering input*: it advances the state by ``dt`` and returns
the corresponding desired foot positions ready to feed into IK.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

from .gait import duty_factor, smoothstep_alpha
from .geometry import DEFAULT_Z, LEGS, NEUTRAL, STEP_HEIGHT, N, Vec3


class CPGNetwork:
    """
    Six coupled Kuramoto oscillators driving a hexapod foot trajectory.

    Attributes
    ----------
    phi
        Length-6 phase vector ``[LF, LM, LB, RF, RM, RB]``, in radians.
    omega
        Natural frequency (rad/s).  Determines the gait that the
        coupling targets blend toward — see :mod:`hexapod_cpg.gait`.
    K_base
        Coupling strength applied to every edge in :data:`EDGES`.
    noise
        Standard deviation of the per-step phase noise injection.
    history
        Deque of ``(t, phi.copy())`` tuples used by the visualiser to
        draw the rolling gait diagram.
    """

    #: Bidirectional coupling graph.  Indices match :data:`hexapod_cpg.geometry.LEGS`.
    EDGES: list[tuple[int, int]] = [
        # ipsilateral, left side
        (0, 1), (1, 2),
        # ipsilateral, right side
        (3, 4), (4, 5),
        # contralateral
        (0, 3), (1, 4), (2, 5),
        # diagonal
        (0, 4), (2, 4), (3, 1), (5, 1),
    ]

    def __init__(
        self,
        omega: float = 2.0,
        K_base: float = 3.5,
        noise: float = 0.04,
        stride: float = 65.0,
        rng: np.random.Generator | None = None,
    ) -> None:
        self._rng = rng if rng is not None else np.random.default_rng()
        self.phi: np.ndarray = self._rng.uniform(0.0, 2 * math.pi, N)
        self.omega: float = float(omega)
        self.K_base: float = float(K_base)
        self.noise: float = float(noise)

        self.moving: bool = False
        self.walk_dir: float = 0.0
        self.turn_rate: float = 0.0
        self.stride: float = float(stride)

        self.history: deque[tuple[float, np.ndarray]] = deque(maxlen=600)
        self.t: float = 0.0

        self._build_coupling()

    # ------------------------------------------------------------------
    # Coupling-matrix setup
    # ------------------------------------------------------------------

    def _build_coupling(self) -> None:
        """Pre-compute the wave and tripod target matrices and the K matrix."""
        # Tripod: groups A={LF, LB, RM}=0°  vs  B={RF, LM, RB}=180°.
        # Note the LEGS indexing: LF=0, LM=1, LB=2, RF=3, RM=4, RB=5.
        group = np.array([0, 1, 0, 1, 0, 1])
        self._theta_tripod = np.zeros((N, N))
        for i in range(N):
            for j in range(N):
                if group[i] != group[j]:
                    self._theta_tripod[i, j] = math.pi

        # Wave gait: 60° front-to-back propagation.
        wave_phase = np.array([0, 1, 2, 3, 4, 5]) * (2 * math.pi / 6.0)
        self._theta_wave = np.zeros((N, N))
        for i in range(N):
            for j in range(N):
                d = wave_phase[j] - wave_phase[i]
                # wrap to (-π, π]
                d = (d + math.pi) % (2 * math.pi) - math.pi
                self._theta_wave[i, j] = d

        # Coupling-strength matrix from the EDGES graph.
        self.K = np.zeros((N, N))
        for i, j in self.EDGES:
            self.K[i, j] = self.K_base
            self.K[j, i] = self.K_base

        self._update_theta()

    def _update_theta(self) -> None:
        """Blend wave and tripod targets according to the current omega."""
        alpha = smoothstep_alpha(self.omega)
        self.theta = (1.0 - alpha) * self._theta_wave + alpha * self._theta_tripod

    # ------------------------------------------------------------------
    # Integration
    # ------------------------------------------------------------------

    def _dphi(self, ph: np.ndarray) -> np.ndarray:
        """Right-hand side of the Kuramoto ODE."""
        d = np.full(N, self.omega)
        for i in range(N):
            coupling = 0.0
            for j in range(N):
                if self.K[i, j] > 0:
                    coupling += self.K[i, j] * math.sin(
                        ph[j] - ph[i] - self.theta[i, j]
                    )
            d[i] += coupling
        return d

    def step(self, dt: float) -> dict[str, Vec3]:
        """
        Advance the network by ``dt`` seconds and return foot targets.

        When ``self.moving`` is ``False`` the phases are frozen; only
        the foot generator is run so the visualiser can keep displaying
        the standing pose.
        """
        if not self.moving:
            return self._feet_from_phases()

        self._update_theta()

        # Explicit midpoint (RK2) for accuracy at 33 Hz frame rate.
        k1 = self._dphi(self.phi)
        k2 = self._dphi(self.phi + 0.5 * dt * k1)
        self.phi += dt * k2
        self.phi += self._rng.standard_normal(N) * self.noise * dt
        self.phi %= 2 * math.pi

        self.t += dt
        self.history.append((self.t, self.phi.copy()))

        return self._feet_from_phases()

    # ------------------------------------------------------------------
    # Foot generator
    # ------------------------------------------------------------------

    def _feet_from_phases(self) -> dict[str, Vec3]:
        """Generate Cartesian foot targets from the current phase vector."""
        feet: dict[str, Vec3] = {}
        duty = duty_factor(self.omega)
        duty_rad = 2 * math.pi * duty

        for idx, leg in enumerate(LEGS):
            phi = self.phi[idx]
            nx, ny = NEUTRAL[leg]
            stride = self.stride

            # Translate stride direction into x/y components.
            sdx = stride * math.cos(self.walk_dir)
            sdy = stride * math.sin(self.walk_dir)

            # Rotational component: tangent to the leg's mount radius.
            leg_r = math.sqrt(nx * nx + ny * ny)
            if leg_r > 1e-3:
                tx = -ny / leg_r * self.turn_rate
                ty = nx / leg_r * self.turn_rate
            else:
                tx = ty = 0.0
            sdx += tx
            sdy += ty

            if phi < duty_rad:
                # Stance: foot stays on the ground, drags backwards.
                t = phi / duty_rad
                foot_x = nx + sdx * (0.5 - t)
                foot_y = ny + sdy * (0.5 - t)
                foot_z = DEFAULT_Z
            else:
                # Swing: foot lifts in a smoothstep arc.
                swing = 2 * math.pi - duty_rad
                t = (phi - duty_rad) / swing
                te = t * t * (3.0 - 2.0 * t)  # smoothstep
                foot_x = nx + sdx * (-0.5 + te)
                foot_y = ny + sdy * (-0.5 + te)
                foot_z = DEFAULT_Z + STEP_HEIGHT * math.sin(math.pi * t)

            feet[leg] = Vec3(foot_x, foot_y, foot_z)

        return feet

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def gait_name(self) -> str:
        """Human-readable gait label (Wave / Ripple / Tripod)."""
        from .gait import gait_label_from_omega
        return gait_label_from_omega(self.omega)

    def phase_coherence(self) -> float:
        """
        Pattern coherence on ``[0, 1]``.

        Measures how closely the actual phase differences track the
        target offsets across all coupled pairs.  Unlike the global
        Kuramoto order parameter :math:`r = \\bigl|\\langle e^{i\\varphi}\\rangle\\bigr|`,
        this metric does *not* collapse on antiphase tripod gaits.
        Returns 1.0 for a perfect gait lock and 0.0 for fully scattered
        phases.
        """
        if not self.moving:
            return 0.0
        errors: list[float] = []
        for i in range(N):
            for j in range(N):
                if self.K[i, j] > 0:
                    actual = (self.phi[j] - self.phi[i] + math.pi) % (
                        2 * math.pi
                    ) - math.pi
                    target = self.theta[i, j]
                    err = abs(actual - target)
                    if err > math.pi:
                        err = 2 * math.pi - err
                    errors.append(err)
        if not errors:
            return 0.0
        return float(1.0 - float(np.mean(errors)) / math.pi)

    def kuramoto_order(self) -> float:
        """
        Classical Kuramoto order parameter
        :math:`r = \\bigl|\\frac{1}{N}\\sum_k e^{i\\varphi_k}\\bigr|`.

        Provided for completeness; prefer :meth:`phase_coherence` for
        gait-quality readouts because it does not collapse on antiphase
        groupings.
        """
        return float(np.abs(np.mean(np.exp(1j * self.phi))))

    # ------------------------------------------------------------------
    # User commands
    # ------------------------------------------------------------------

    def perturb(self, leg_idx: int = 0, amount: float = math.pi) -> None:
        """Kick a single oscillator's phase to test re-synchronisation."""
        self.phi[leg_idx] = (self.phi[leg_idx] + amount) % (2 * math.pi)

    def randomise_phases(self) -> None:
        """Resample every phase from the uniform distribution."""
        self.phi = self._rng.uniform(0.0, 2 * math.pi, N)

    def set_speed(self, omega: float) -> None:
        """Set the natural-frequency drive (clamped to ``[0.5, 6.0]``)."""
        self.omega = float(np.clip(omega, 0.5, 6.0))

    def set_walking(self, direction_deg: float) -> None:
        """
        Switch into walking mode with a heading direction in degrees
        (0 = forward, 180 = backward).
        """
        self.walk_dir = math.radians(direction_deg)
        self.turn_rate = 0.0
        self.moving = True

    def set_turning(self, direction: float) -> None:
        """
        Switch into turning mode.  ``direction = +1`` rotates left,
        ``-1`` rotates right; magnitude scales the rotational stride.
        """
        self.walk_dir = 0.0
        self.turn_rate = 55.0 * float(direction)
        self.moving = True

    def stop(self) -> None:
        """Freeze the oscillators (legs hold their current foot positions)."""
        self.moving = False
