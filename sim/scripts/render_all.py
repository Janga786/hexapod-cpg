"""
Run every renderer in order, regenerating the full figure and
animation gallery.

Usage::

    python sim/scripts/render_all.py
"""

from __future__ import annotations

import runpy
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
RENDERERS: list[str] = [
    "render_coupling_graph.py",
    "render_omega_sweep.py",
    "render_workspace_map.py",
    "render_oscillator_timeseries.py",
    "render_gait_comparison.py",
    "render_foot_trajectories.py",
    "render_phase_wheel_animation.py",
    "render_3d_walk_animation.py",
]


def main() -> None:
    overall_t0 = time.perf_counter()
    for filename in RENDERERS:
        print()
        print("#" * 76)
        print(f"# {filename}")
        print("#" * 76)
        t0 = time.perf_counter()
        runpy.run_path(str(HERE / filename), run_name="__main__")
        print(f"\n[ {filename} : {time.perf_counter() - t0:.1f} s ]")

    print()
    print("=" * 76)
    print(f"all renderers completed in {time.perf_counter() - overall_t0:.1f} s")
    print("=" * 76)


if __name__ == "__main__":
    main()
