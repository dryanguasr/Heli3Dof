import numpy as np
from heli3dof.kinematics import f_kine_heli3dof


def test_zero_angles_positions():
    L_A = 0.1
    L_B = 1.0
    L_C = 1.0
    pos = f_kine_heli3dof(0.0, 0.0, 0.0, L_A, L_B, L_C)
    np.testing.assert_allclose(pos["motor_R"], [L_B/2, L_A, 0.75*L_C])
    np.testing.assert_allclose(pos["motor_L"], [L_B/2, -L_A, 0.75*L_C])
    np.testing.assert_allclose(pos["cm_A"], [L_B/2, 0.0, 0.75*L_C])
    np.testing.assert_allclose(pos["cp"], [-L_B/2, 0.0, 0.75*L_C])
    np.testing.assert_allclose(pos["base"], [0.0, 0.0, 0.75*L_C])
