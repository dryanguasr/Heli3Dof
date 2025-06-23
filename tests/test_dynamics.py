import numpy as np
from heli3dof.dynamics import heli3dof_dynamics


def test_dynamics_zero_input():
    x = np.zeros(6)
    u = np.zeros(2)
    dx = heli3dof_dynamics(0.0, x, u)
    assert dx[0] == 0
    assert dx[1] == 0
    assert dx[2] == 0
    assert dx[4] == 0
    assert dx[5] == 0
    # pitch acceleration is due to gravity difference
    assert np.isclose(dx[3], -(2.5 - 2.0) * 9.8)
