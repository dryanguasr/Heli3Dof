import numpy as np

from heli3dof.control import fuzzy_pd_torque, fuzzy_pitch_roll_controller


def test_fuzzy_pd_zero_error_returns_zero():
    assert np.isclose(fuzzy_pd_torque(0.0, 0.0), 0.0)


def test_fuzzy_pd_sign():
    assert fuzzy_pd_torque(0.8, 0.0) > 0.0
    assert fuzzy_pd_torque(-0.8, 0.0) < 0.0


def test_pitch_roll_controller_shape():
    state = np.zeros(6)
    u = fuzzy_pitch_roll_controller(state, references=(0.1, -0.1), hover_force=1.0)
    assert u.shape == (2,)
    assert np.all(np.isfinite(u))
