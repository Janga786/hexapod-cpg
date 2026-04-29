"""
hexapod_cpg
===========

A central pattern generator (CPG) for hexapod locomotion built on a
network of six Kuramoto-style coupled phase oscillators, paired with a
geometry-faithful 3-DoF leg kinematic model and a real-time matplotlib
visualisation that exposes the dynamics of the CPG itself (phase
wheel, gait diagram, order parameter) — not just the resulting motion.

The package implements, from first principles:

- An 18-DoF analytical inverse-kinematics solver for a hexagonal hexapod
  body with 6 × {coxa, femur, tibia} legs, including reach checks and
  servo-limit validation.
- A 6-oscillator Kuramoto network whose target phase-offset matrix
  smoothly interpolates between the **wave**, **ripple** and **tripod**
  gait patterns as a function of the natural frequency :math:`\\omega`.
- A foot trajectory generator that synthesises Cartesian foot poses
  from CPG phases (cosine-blended stance, smoothstep swing, sinusoidal
  step lift) with online translation- and rotation-velocity inputs.
- Singularity-aware diagnostics: a custom *gait coherence* metric that
  measures how closely actual phase differences match the target
  offsets across all coupled pairs, and that does not collapse on
  antiphase tripod gaits the way the global Kuramoto :math:`r` does.
- A polished interactive 3-D visualiser with a phase-wheel, a rolling
  gait-diagram and a live status panel — all driven from one
  ``FuncAnimation`` loop.

Public API
----------

``Geometry & inverse kinematics``
    :data:`COXA_LENGTH`, :data:`FEMUR_LENGTH`, :data:`TIBIA_LENGTH`,
    :data:`MOUNTS`, :data:`NEUTRAL`, :data:`LEGS`, :data:`BODY_POLY`,
    :class:`JointAngles`, :class:`Vec3`, :func:`ik`, :func:`joint_pts`.

``CPG dynamics``
    :class:`CPGNetwork`.

``Visualisation``
    :class:`CPGViz`, :func:`pick_backend`.
"""

from __future__ import annotations

from .colors import BG, GRID_COL, LEG_COLS, PANEL_BG, gait_colour
from .cpg import CPGNetwork
from .gait import gait_label_from_omega
from .geometry import (
    BODY_POLY,
    COXA_LENGTH,
    DEFAULT_Z,
    FEMUR_LENGTH,
    LEGS,
    MAX_REACH,
    MIN_REACH,
    MOUNTS,
    NEUTRAL,
    SERVO_MAX,
    SERVO_MIN,
    STEP_HEIGHT,
    TIBIA_LENGTH,
    TIBIA_OFFSET,
    JointAngles,
    N,
    Vec3,
    ik,
    joint_pts,
)
from .viz import CPGViz, pick_backend

__all__ = [
    # geometry
    "COXA_LENGTH",
    "FEMUR_LENGTH",
    "TIBIA_LENGTH",
    "TIBIA_OFFSET",
    "DEFAULT_Z",
    "STEP_HEIGHT",
    "MAX_REACH",
    "MIN_REACH",
    "SERVO_MIN",
    "SERVO_MAX",
    "MOUNTS",
    "NEUTRAL",
    "LEGS",
    "BODY_POLY",
    "N",
    "JointAngles",
    "Vec3",
    "ik",
    "joint_pts",
    # cpg
    "CPGNetwork",
    # gait
    "gait_label_from_omega",
    # colors
    "LEG_COLS",
    "BG",
    "PANEL_BG",
    "GRID_COL",
    "gait_colour",
    # visualisation
    "CPGViz",
    "pick_backend",
]

__version__ = "1.0.0"
__author__ = "Bliss Janga"
__license__ = "MIT"
