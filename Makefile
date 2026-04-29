# =============================================================================
#  Hexapod CPG — top-level Makefile
# =============================================================================

PY        ?= python3
PIP       ?= $(PY) -m pip
PYTEST    ?= $(PY) -m pytest

.PHONY: help install dev test lint figures animations gallery sim clean

help:  ## show this help
	@awk 'BEGIN{FS=":.*##"; printf "Targets:\n"} /^[a-zA-Z_-]+:.*?##/ {printf "  \033[1;36m%-12s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

install:  ## install the hexapod_cpg package (editable) + runtime deps
	$(PIP) install -e .

dev:  ## install the package + dev/test dependencies
	$(PIP) install -e .[dev]

test:  ## run the full pytest suite
	$(PYTEST) -q

lint:  ## ruff lint over the package, scripts, and tests
	$(PY) -m ruff check sim

sim:  ## launch the interactive CPG dashboard
	$(PY) sim/scripts/run_interactive.py

figures:  ## regenerate every static figure
	$(PY) sim/scripts/render_coupling_graph.py
	$(PY) sim/scripts/render_omega_sweep.py
	$(PY) sim/scripts/render_workspace_map.py
	$(PY) sim/scripts/render_oscillator_timeseries.py
	$(PY) sim/scripts/render_gait_comparison.py
	$(PY) sim/scripts/render_foot_trajectories.py

animations:  ## regenerate every animated GIF
	$(PY) sim/scripts/render_phase_wheel_animation.py
	$(PY) sim/scripts/render_3d_walk_animation.py

gallery:  ## regenerate the entire figure / animation gallery
	$(PY) sim/scripts/render_all.py

clean:  ## remove build / cache artefacts (keeps figures & animations)
	rm -rf .pytest_cache .ruff_cache .mypy_cache build dist *.egg-info
	find . -type d -name __pycache__ -prune -exec rm -rf {} +
