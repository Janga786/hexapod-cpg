"""Resolve the ``figures/`` and ``animations/`` directories."""

from __future__ import annotations

from pathlib import Path

REPO_ROOT: Path = Path(__file__).resolve().parents[2]
FIG_DIR: Path = REPO_ROOT / "figures"
ANIM_DIR: Path = REPO_ROOT / "animations"

for _d in (FIG_DIR, ANIM_DIR):
    _d.mkdir(parents=True, exist_ok=True)
