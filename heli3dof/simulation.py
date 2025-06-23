from __future__ import annotations

import numpy as np
from .dynamics import heli3dof_dynamics


def simulate(initial_state: np.ndarray, controller, t: np.ndarray,
             params: dict | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Simulate the helicopter using Euler integration."""
    x = np.zeros((len(t), len(initial_state)))
    u = np.zeros((len(t), 2))
    x[0] = initial_state
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        u[i] = controller(t[i - 1], x[i - 1])
        dx = heli3dof_dynamics(t[i - 1], x[i - 1], u[i], params)
        x[i] = x[i - 1] + dx * dt
    return x, u
