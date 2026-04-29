"""
Project-wide colour palette
===========================

A single source of truth for every colour used in figures and
animations, so that batch-generated artwork retains a coherent visual
identity across plots.
"""

from __future__ import annotations

import numpy as np

from .gait import smoothstep_alpha

# Backgrounds and grid lines.
BG: str = "#0a0a1a"
PANEL_BG: str = "#0d0d1f"
GRID_COL: str = "#1e1e3a"

# Per-leg colours (cool/blue for left, warm/red for right; brightness
# decreases front → back).
LEG_COLS: dict[str, str] = {
    "LF": "#4fc3f7", "LM": "#29b6f6", "LB": "#0288d1",
    "RF": "#ef9a9a", "RM": "#e57373", "RB": "#c62828",
}

# Link colours (used in 3-D renders).
COL_COXA: str = "#3498db"
COL_FEMUR: str = "#2ecc71"
COL_TIBIA: str = "#e74c3c"
COL_JOINT: str = "#ecf0f1"
COL_FOOT_DOWN: str = "#f39c12"
COL_FOOT_UP: str = "#9b59b6"
COL_BODY: str = "#2c3e50"


def gait_colour(omega: float) -> str:
    """
    Colour-map omega to a `#rrggbb` string.

    Smoothly traverses blue (low ω, wave) → green (mid ω, ripple) →
    orange (high ω, tripod) using the same smoothstep alpha that the
    CPG uses to blend its target offsets, so the colour at the title
    bar always agrees with the actual gait pattern.
    """
    a = float(smoothstep_alpha(omega))
    if a < 0.5:
        t = a * 2.0
        r = int(0 + t * 50)
        g = int(100 + t * 156)
        b = int(220 - t * 20)
    else:
        t = (a - 0.5) * 2.0
        r = int(50 + t * 205)
        g = int(256 - t * 106)
        b = int(200 - t * 200)
    r = int(np.clip(r, 0, 255))
    g = int(np.clip(g, 0, 255))
    b = int(np.clip(b, 0, 255))
    return f"#{r:02x}{g:02x}{b:02x}"
