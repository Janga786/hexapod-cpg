"""
Hexapod geometry and analytical leg kinematics
==============================================

The body is a regular hexagonal frame whose six coxa servos are mounted
at the body-frame positions in :data:`MOUNTS`, with neutral foot
positions in :data:`NEUTRAL`.  Each leg is a 3-DoF serial chain:

- **Coxa** rotates about a vertical axis at the mount.
- **Femur** rotates about a horizontal axis perpendicular to the coxa.
- **Tibia** rotates parallel to the femur axis.

Link constants are taken from the physical robot CAD (28 mm coxa,
84 mm femur, 127 mm tibia, ``TIBIA_OFFSET = 14°`` mechanical offset of
the tibia relative to the femur axis at zero command).

Inverse kinematics
------------------

For a desired foot position :math:`(x, y, z)` in the body frame, the
solver computes:

.. math::

   \\mathrm{coxa}    &= \\operatorname{atan2}(\\Delta y, \\Delta x) - \\theta_m \\\\
   \\rho             &= \\sqrt{\\Delta x^2 + \\Delta y^2} - \\ell_c \\\\
   D                 &= \\sqrt{\\rho^2 + z^2}                              \\\\
   \\mathrm{femur}   &= \\arccos\\!\\left(\\tfrac{D^2 + \\ell_f^2 - \\ell_t^2}{2 D \\ell_f}\\right)
                       + \\operatorname{atan2}(z, \\rho) \\\\
   \\mathrm{tibia}   &= \\arccos\\!\\left(
                          \\tfrac{\\ell_f^2 + \\ell_t^2 - D^2}{2 \\ell_f \\ell_t}
                         \\right) - 90^\\circ - \\beta_{\\mathrm{off}}

with reach checks against :data:`MAX_REACH` / :data:`MIN_REACH` and
servo-range checks against :data:`SERVO_MIN` / :data:`SERVO_MAX`.
The :attr:`JointAngles.valid` flag distinguishes a feasible solution
from one that fell outside the workspace.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

# ---------------------------------------------------------------------------
# Link constants (millimetres) — physical CAD measurements
# ---------------------------------------------------------------------------

COXA_LENGTH: float = 28.0    # coxa pivot → femur pivot
FEMUR_LENGTH: float = 84.0   # femur pivot → tibia pivot
TIBIA_LENGTH: float = 127.0  # tibia pivot → foot tip


# ---------------------------------------------------------------------------
# Body geometry — coxa-mount positions and neutral foot positions, mm
# ---------------------------------------------------------------------------

#: Coxa-servo mount positions in the body frame ``(x_forward, y_left)``.
MOUNTS: dict[str, tuple[float, float]] = {
    "LF": (74.0, 39.0),  "LM": (0.0, 64.0),   "LB": (-74.0, 39.0),
    "RF": (74.0, -39.0), "RM": (0.0, -64.0),  "RB": (-74.0, -39.0),
}

#: Default standing foot positions, used as the centre of each foot's
#: stance / swing cycle.
NEUTRAL: dict[str, tuple[float, float]] = {
    "LF": (200.3, 105.5),  "LM": (0.0, 206.7),   "LB": (-200.3, 105.5),
    "RF": (200.3, -105.5), "RM": (0.0, -206.7),  "RB": (-200.3, -105.5),
}


# ---------------------------------------------------------------------------
# Servo limits and walking parameters
# ---------------------------------------------------------------------------

SERVO_MIN: float = -90.0
SERVO_MAX: float = 90.0

#: Mechanical offset of the tibia at the zero-command pose (degrees).
TIBIA_OFFSET: float = 14.0

#: Foot height when in stance (millimetres, body frame).
DEFAULT_Z: float = -123.2

#: Peak foot lift during swing (millimetres).
STEP_HEIGHT: float = 38.0

#: Maximum reach of the femur+tibia 2-link chain.
MAX_REACH: float = FEMUR_LENGTH + TIBIA_LENGTH

#: Minimum reach (folded) of the femur+tibia 2-link chain.
MIN_REACH: float = abs(FEMUR_LENGTH - TIBIA_LENGTH)


# ---------------------------------------------------------------------------
# Leg ordering and body polygon
# ---------------------------------------------------------------------------

#: Canonical leg order (LF/LM/LB/RF/RM/RB).  Indices in this list match
#: the columns/rows of every coupling matrix used in :mod:`hexapod_cpg.cpg`.
LEGS: list[str] = ["LF", "LM", "LB", "RF", "RM", "RB"]

#: Vertex order for a closed body polyline ``(LF→RF→RM→RB→LB→LM→LF)``.
BODY_POLY: list[str] = ["LF", "RF", "RM", "RB", "LB", "LM"]

#: Number of legs (and oscillators).  Convenience constant.
N: int = 6


# ---------------------------------------------------------------------------
# Lightweight value types
# ---------------------------------------------------------------------------

@dataclass
class JointAngles:
    """
    Result of a single-leg inverse-kinematics call.

    ``coxa``, ``femur``, ``tibia`` are degrees; ``valid`` is ``True``
    only if the requested foot position is reachable *and* the
    resulting joint angles fall inside ``[SERVO_MIN, SERVO_MAX]``.
    """

    coxa: float = 0.0
    femur: float = 0.0
    tibia: float = 0.0
    valid: bool = False


@dataclass
class Vec3:
    """A simple 3-vector in millimetres."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


# ---------------------------------------------------------------------------
# Inverse kinematics (single leg)
# ---------------------------------------------------------------------------

def ik(foot: Vec3, mx: float, my: float) -> JointAngles:
    """
    Solve the 3-DoF inverse kinematics for a single hexapod leg.

    Parameters
    ----------
    foot
        Desired foot tip position in the body frame, millimetres.
    mx, my
        ``(x, y)`` of the coxa-mount on the body frame, millimetres.

    Returns
    -------
    angles
        The :class:`JointAngles` result.  Caller must check
        ``angles.valid`` before applying — the method silently returns
        an invalid result when the target is unreachable, so it can be
        chained inside an animation loop without exceptions.
    """
    out = JointAngles()
    dx = foot.x - mx
    dy = foot.y - my
    dz = foot.z

    mount_angle = math.atan2(my, mx)
    foot_angle = math.atan2(dy, dx)

    # Coxa: angle from mount-radial direction to target, wrapped to (-180°, 180°].
    coxa_deg = math.degrees(foot_angle - mount_angle)
    while coxa_deg > 180.0:
        coxa_deg -= 360.0
    while coxa_deg < -180.0:
        coxa_deg += 360.0
    out.coxa = coxa_deg

    # Reduce the 3-D problem to the planar 2R triangle (femur + tibia).
    horizontal = math.sqrt(dx * dx + dy * dy) - COXA_LENGTH
    vertical = dz
    distance = math.sqrt(horizontal * horizontal + vertical * vertical)
    if distance > MAX_REACH or distance < MIN_REACH or distance < 1e-3:
        return out

    pitch_to_target = math.atan2(vertical, horizontal)

    cos_femur = max(
        -1.0,
        min(
            1.0,
            (distance ** 2 + FEMUR_LENGTH ** 2 - TIBIA_LENGTH ** 2)
            / (2.0 * distance * FEMUR_LENGTH),
        ),
    )
    out.femur = math.degrees(math.acos(cos_femur) + pitch_to_target)

    cos_tibia = max(
        -1.0,
        min(
            1.0,
            (FEMUR_LENGTH ** 2 + TIBIA_LENGTH ** 2 - distance ** 2)
            / (2.0 * FEMUR_LENGTH * TIBIA_LENGTH),
        ),
    )
    out.tibia = math.degrees(math.acos(cos_tibia)) - 90.0 - TIBIA_OFFSET

    if not all(SERVO_MIN <= a <= SERVO_MAX for a in (out.coxa, out.femur, out.tibia)):
        return out
    if any(math.isnan(a) for a in (out.coxa, out.femur, out.tibia)):
        return out

    out.valid = True
    return out


def joint_pts(
    angles: JointAngles, mx: float, my: float
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """
    Forward kinematics of a single leg, returning four 3-D joint
    positions in the body frame: ``mount``, ``coxa-end``, ``femur-end``
    and ``foot-tip`` (in that order).
    """
    mount_angle = math.atan2(my, mx)
    coxa_world_angle = math.radians(angles.coxa) + mount_angle
    femur_angle = math.radians(angles.femur)
    tibia_angle = math.radians(angles.tibia + 90.0 + TIBIA_OFFSET)

    coxa_end = np.array(
        [mx + COXA_LENGTH * math.cos(coxa_world_angle),
         my + COXA_LENGTH * math.sin(coxa_world_angle),
         0.0]
    )

    femur_h = FEMUR_LENGTH * math.cos(femur_angle)
    femur_v = FEMUR_LENGTH * math.sin(femur_angle)
    femur_end = np.array(
        [coxa_end[0] + femur_h * math.cos(coxa_world_angle),
         coxa_end[1] + femur_h * math.sin(coxa_world_angle),
         femur_v]
    )

    tibia_world = femur_angle - (math.pi - tibia_angle)
    tibia_h = femur_h + TIBIA_LENGTH * math.cos(tibia_world)
    tibia_v = femur_v + TIBIA_LENGTH * math.sin(tibia_world)
    foot_tip = np.array(
        [mx + (COXA_LENGTH + tibia_h) * math.cos(coxa_world_angle),
         my + (COXA_LENGTH + tibia_h) * math.sin(coxa_world_angle),
         tibia_v]
    )

    mount = np.array([mx, my, 0.0])
    return mount, coxa_end, femur_end, foot_tip
