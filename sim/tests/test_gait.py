"""Tests for the gait taxonomy / labelling helpers."""

from __future__ import annotations

import pytest
from hexapod_cpg.gait import duty_factor, gait_label_from_omega, smoothstep_alpha


@pytest.mark.parametrize(
    "omega, expected",
    [
        (0.5,  "Wave"),
        (1.0,  "Wave"),
        (3.0,  "Ripple"),
        (4.5,  "Tripod"),
        (5.5,  "Tripod"),
    ],
)
def test_gait_label_at_canonical_speeds(omega, expected):
    assert gait_label_from_omega(omega) == expected


def test_alpha_clamped_to_unit_interval():
    """smoothstep_alpha must clamp to [0,1] outside the operating range."""
    assert smoothstep_alpha(-1.0) == 0.0
    assert smoothstep_alpha(0.0) == 0.0
    assert smoothstep_alpha(1.0) == 0.0
    assert smoothstep_alpha(5.0) == 1.0
    assert smoothstep_alpha(6.0) == 1.0
    assert smoothstep_alpha(100.0) == 1.0


def test_alpha_monotonic():
    """α must be non-decreasing in ω."""
    omegas = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]
    alphas = [smoothstep_alpha(o) for o in omegas]
    for a, b in zip(alphas, alphas[1:], strict=False):
        assert a <= b + 1e-12


def test_duty_factor_decreases_with_speed():
    """Faster ω ⇒ shorter duty factor."""
    assert duty_factor(1.0) > duty_factor(5.0)


def test_duty_factor_clamped_below():
    """duty_factor should never go below 0.50."""
    assert duty_factor(10.0) >= 0.50


def test_duty_factor_clamped_above():
    """duty_factor should never exceed 0.72."""
    assert duty_factor(0.0) <= 0.72
