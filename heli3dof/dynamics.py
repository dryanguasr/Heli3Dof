import numpy as np


def heli3dof_dynamics(t: float, x: np.ndarray, u: np.ndarray,
                      params: dict | None = None) -> np.ndarray:
    """Dynamics of the 3-DOF helicopter.

    Parameters
    ----------
    t : float
        Current time (unused but included for compatibility).
    x : ndarray, shape (6,)
        State vector [theta, theta_dot, phi, phi_dot, psi, psi_dot].
    u : ndarray, shape (2,)
        Control inputs [front_motor_force, back_motor_force].
    params : dict, optional
        Dictionary with model parameters. If None, default parameters are
        used.

    Returns
    -------
    ndarray, shape (6,)
        Time derivative of the state.
    """
    if params is None:
        params = {
            "Jx": 1.0,
            "Jy": 1.0,
            "Jz": 1.0,
            "R": 0.25,
            "L": 0.5,
            "mcp": 2.0,
            "mm": 2.5,
            "g": 9.8,
        }

    Jx = params["Jx"]
    Jy = params["Jy"]
    Jz = params["Jz"]
    R = params["R"]
    L = params["L"]
    mcp = params["mcp"]
    mm = params["mm"]
    g = params["g"]

    dx = np.zeros_like(x)
    dx[0] = x[1]
    dx[1] = (u[0] - u[1]) / (2 * Jx * R)
    dx[2] = x[3]
    dx[3] = (u[0] + u[1]) / (Jy * L) * np.cos(x[0]) - (mm - mcp) * g / Jy * np.cos(x[2])
    dx[4] = x[5]
    dx[5] = (u[0] + u[1]) / (Jz * L) * np.sin(x[0]) * np.cos(x[2])

    return dx
