import numpy as np


def inertia_yaw_heli3dof(pos: dict, m_motor: float, m_green: float, m_red: float) -> float:
    """Compute the yaw inertia of the helicopter."""

    def rod_inertia(p1: np.ndarray, p2: np.ndarray, m: float) -> float:
        return (m / 3) * (np.dot(p1, p1) + np.dot(p1, p2) + np.dot(p2, p2))

    o = pos["base"][:2]
    rR = pos["motor_R"][:2] - o
    rL = pos["motor_L"][:2] - o
    r_cp = pos["cp"][:2] - o
    r_cm_A = pos["cm_A"][:2] - o

    Jz = (
        m_motor * (rR[0] ** 2 + rR[1] ** 2)
        + m_motor * (rL[0] ** 2 + rL[1] ** 2)
        + rod_inertia(r_cp, r_cm_A, m_green)
        + rod_inertia(rL, rR, m_red)
    )
    return Jz
