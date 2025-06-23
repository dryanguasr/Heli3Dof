import numpy as np
from .utils import rot_x, rot_y, rot_z


def f_kine_heli3dof(theta: float, alpha: float, psi: float,
                    L_A: float, L_B: float, L_C: float) -> dict:
    """Direct kinematics of the 3-DOF helicopter."""
    Rz = rot_z(psi)
    Ry = rot_y(alpha)
    green_vector = Rz @ Ry @ np.array([1.0, 0.0, 0.0])

    pivot_green = np.array([0.0, 0.0, 0.75 * L_C])

    green_start = pivot_green - (L_B / 2) * green_vector
    green_end = pivot_green + (L_B / 2) * green_vector

    red_axis = np.array([0.0, 1.0, 0.0])
    Rx = rot_x(theta)
    arm_vector = Rz @ Ry @ Rx @ red_axis

    pos = {}
    pos["motor_R"] = green_end + L_A * arm_vector
    pos["motor_L"] = green_end - L_A * arm_vector
    pos["cm_A"] = green_end
    pos["cm_B"] = pivot_green
    pos["cp"] = green_start
    pos["base"] = pivot_green
    return pos
