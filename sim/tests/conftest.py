"""Shared pytest fixtures and headless-matplotlib hookup."""

from __future__ import annotations

import matplotlib
import numpy as np
import pytest

# Force a non-interactive backend for the entire test run so the
# rendering scripts work in CI without a display.
matplotlib.use("Agg")


@pytest.fixture(scope="session")
def rng() -> np.random.Generator:
    """A deterministic RNG shared across the whole test session."""
    return np.random.default_rng(2025)
