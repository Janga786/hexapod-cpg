"""
Gait taxonomy and labelling
===========================

Three gaits are exposed by the CPG depending on the natural-frequency
drive :math:`\\omega`:

============ =================== ============== ===========
Gait         Phase pattern       Duty factor    Stable for
============ =================== ============== ===========
Wave         60° front-to-back   ~ 0.83         low ω
Ripple       intermediate        ~ 0.67         mid ω
Tripod       two anti-phase      ~ 0.50         high ω
             groups
============ =================== ============== ===========

The transition between targets is a smoothstep on
:math:`\\alpha = \\mathrm{clamp}((\\omega - 1) / 4, 0, 1)`.  Below
:math:`\\alpha = 0.25` the gait reads "Wave"; below 0.65 "Ripple";
otherwise "Tripod".  This module provides the labelling helper that
both the visualiser and the unit tests use.
"""

from __future__ import annotations

import numpy as np


def smoothstep_alpha(omega: float) -> float:
    """
    Compute the smoothstep blend coefficient between wave (α=0) and
    tripod (α=1) for a given natural frequency ``omega`` (rad/s).
    """
    a = float(np.clip((omega - 1.0) / 4.0, 0.0, 1.0))
    return a * a * (3.0 - 2.0 * a)


def gait_label_from_omega(omega: float) -> str:
    """Return the human-readable gait label for ``omega``."""
    a = smoothstep_alpha(omega)
    if a < 0.25:
        return "Wave"
    if a < 0.65:
        return "Ripple"
    return "Tripod"


def duty_factor(omega: float) -> float:
    """
    Return the gait duty factor for ``omega``.

    The duty factor is the fraction of one cycle that a foot spends in
    stance; it falls from ~0.72 at low ω to ~0.50 at high ω, modelling
    the empirical observation that fast hexapods spend less time on
    each foot.
    """
    return float(np.clip(0.70 - 0.08 * (omega - 1.0) / 4.0, 0.50, 0.72))
