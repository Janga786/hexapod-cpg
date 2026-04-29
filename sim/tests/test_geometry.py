"""Tests for the leg geometry / inverse kinematics module."""

from __future__ import annotations

import numpy as np
import pytest
from hexapod_cpg import (
    COXA_LENGTH,
    DEFAULT_Z,
    FEMUR_LENGTH,
    LEGS,
    MAX_REACH,
    MOUNTS,
    NEUTRAL,
    TIBIA_LENGTH,
    Vec3,
    ik,
    joint_pts,
)


def test_constants_make_sense():
    assert COXA_LENGTH > 0
    assert FEMUR_LENGTH > 0
    assert TIBIA_LENGTH > 0
    assert MAX_REACH == FEMUR_LENGTH + TIBIA_LENGTH
    assert len(MOUNTS) == 6
    assert set(MOUNTS) == set(LEGS)
    assert set(NEUTRAL) == set(LEGS)


def test_ik_neutral_pose_is_valid():
    """Every leg at its NEUTRAL foot at DEFAULT_Z must be reachable."""
    for leg in LEGS:
        nx, ny = NEUTRAL[leg]
        mx, my = MOUNTS[leg]
        ang = ik(Vec3(nx, ny, DEFAULT_Z), mx, my)
        assert ang.valid, f"leg {leg} invalid at neutral pose"


def test_ik_unreachable_target_marked_invalid():
    """A target far beyond MAX_REACH must be flagged invalid (no exception)."""
    ang = ik(Vec3(10000.0, 0.0, 0.0), 0.0, 0.0)
    assert not ang.valid


def test_ik_then_fk_round_trip():
    """IK→FK should reproduce the foot tip for every reachable target."""
    targets = [
        ("LF", NEUTRAL["LF"][0],     NEUTRAL["LF"][1],     DEFAULT_Z),
        ("LM", NEUTRAL["LM"][0] - 5, NEUTRAL["LM"][1] + 3, DEFAULT_Z + 10),
        ("RB", NEUTRAL["RB"][0] + 2, NEUTRAL["RB"][1] - 4, DEFAULT_Z - 8),
    ]
    for leg, x, y, z in targets:
        mx, my = MOUNTS[leg]
        ang = ik(Vec3(x, y, z), mx, my)
        assert ang.valid
        _, _, _, foot_tip = joint_pts(ang, mx, my)
        np.testing.assert_allclose(foot_tip, [x, y, z], atol=1e-3)


def test_joint_pts_returns_four_3d_points():
    leg = "RM"
    mx, my = MOUNTS[leg]
    nx, ny = NEUTRAL[leg]
    ang = ik(Vec3(nx, ny, DEFAULT_Z), mx, my)
    pts = joint_pts(ang, mx, my)
    assert len(pts) == 4
    for p in pts:
        assert p.shape == (3,)


@pytest.mark.parametrize("leg", LEGS)
def test_swing_apex_is_reachable(leg):
    """The swing-arc apex (foot fully lifted by STEP_HEIGHT) must be reachable."""
    nx, ny = NEUTRAL[leg]
    mx, my = MOUNTS[leg]
    ang = ik(Vec3(nx, ny, DEFAULT_Z + 38), mx, my)
    assert ang.valid


def test_ik_outside_servo_range_marked_invalid():
    """A foot pulled directly above the mount has impossible coxa angle."""
    leg = "LF"
    mx, my = MOUNTS[leg]
    # Force a target where the femur+tibia chain is folded too tight
    ang = ik(Vec3(mx, my, -10.0), mx, my)
    assert not ang.valid
