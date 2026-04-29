"""Tests for the Kuramoto-CPG dynamics."""

from __future__ import annotations

import math

import numpy as np
from hexapod_cpg import LEGS, CPGNetwork


def test_initial_phases_in_range():
    cpg = CPGNetwork(rng=np.random.default_rng(1))
    assert cpg.phi.shape == (6,)
    assert np.all(cpg.phi >= 0.0)
    assert np.all(cpg.phi < 2 * math.pi)


def test_step_returns_one_vec_per_leg():
    cpg = CPGNetwork(rng=np.random.default_rng(2))
    cpg.set_walking(0.0)
    feet = cpg.step(0.02)
    assert set(feet.keys()) == set(LEGS)


def test_phases_stay_wrapped_after_many_steps():
    cpg = CPGNetwork(omega=4.0, rng=np.random.default_rng(3))
    cpg.set_walking(0.0)
    for _ in range(500):
        cpg.step(0.02)
    assert np.all(cpg.phi >= 0.0)
    assert np.all(cpg.phi < 2 * math.pi)


def test_coupling_matrix_is_symmetric_in_K():
    """Coupling strengths must be symmetric (undirected coupling graph)."""
    cpg = CPGNetwork()
    np.testing.assert_array_equal(cpg.K, cpg.K.T)


def test_coupling_matrix_off_diagonals_only():
    """No self-coupling: K[i,i] must be zero."""
    cpg = CPGNetwork()
    assert np.all(np.diag(cpg.K) == 0.0)


def test_tripod_target_is_antiphase():
    """In the tripod target matrix, paired groups must be π apart."""
    cpg = CPGNetwork()
    # group A = {0,2,4} = {LF,LB,RM}; group B = {1,3,5} = {LM,RF,RB}
    A, B = (0, 2, 4), (1, 3, 5)
    for i in A:
        for j in B:
            assert math.isclose(cpg._theta_tripod[i, j], math.pi, rel_tol=1e-9)
            assert math.isclose(cpg._theta_tripod[j, i], math.pi, rel_tol=1e-9)
        for j in A:
            assert cpg._theta_tripod[i, j] == 0.0


def test_high_omega_locks_into_high_coherence():
    """At ω=5, after a few seconds the pattern coherence should be close to 1."""
    cpg = CPGNetwork(omega=5.0, rng=np.random.default_rng(7))
    cpg.set_walking(0.0)
    for _ in range(int(5.0 / 0.02)):
        cpg.step(0.02)
    assert cpg.phase_coherence() > 0.85


def test_low_omega_settles_into_wave():
    """At ω=1 we expect monotonic spacing of phases (wave gait)."""
    cpg = CPGNetwork(omega=1.0, rng=np.random.default_rng(9))
    cpg.set_walking(0.0)
    for _ in range(int(8.0 / 0.02)):
        cpg.step(0.02)
    assert cpg.phase_coherence() > 0.6


def test_perturb_breaks_coherence_then_recovers():
    """A π perturbation should drop coherence; CPG should recover."""
    cpg = CPGNetwork(omega=5.0, rng=np.random.default_rng(11))
    cpg.set_walking(0.0)
    for _ in range(int(5.0 / 0.02)):
        cpg.step(0.02)
    locked = cpg.phase_coherence()
    cpg.perturb(0, math.pi)
    after_kick = cpg.phase_coherence()
    assert after_kick < locked
    for _ in range(int(4.0 / 0.02)):
        cpg.step(0.02)
    recovered = cpg.phase_coherence()
    assert recovered > after_kick


def test_stop_freezes_phases():
    cpg = CPGNetwork(omega=5.0, rng=np.random.default_rng(13))
    cpg.set_walking(0.0)
    for _ in range(50):
        cpg.step(0.02)
    cpg.stop()
    snapshot = cpg.phi.copy()
    for _ in range(50):
        cpg.step(0.02)
    np.testing.assert_array_equal(cpg.phi, snapshot)


def test_set_speed_clamps_to_range():
    cpg = CPGNetwork()
    cpg.set_speed(100.0)
    assert cpg.omega <= 6.0
    cpg.set_speed(-100.0)
    assert cpg.omega >= 0.5


def test_kuramoto_order_is_in_unit_interval():
    cpg = CPGNetwork(rng=np.random.default_rng(17))
    cpg.set_walking(0.0)
    for _ in range(50):
        cpg.step(0.02)
    r = cpg.kuramoto_order()
    assert 0.0 <= r <= 1.0
