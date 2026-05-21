from __future__ import annotations

import numpy as np


def _finite_difference_jacobian(fun, x: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    f0 = np.asarray(fun(x), dtype=float)
    jac = np.zeros((f0.size, x.size), dtype=float)
    for i in range(x.size):
        dx = np.zeros_like(x)
        dx[i] = eps
        jac[:, i] = (np.asarray(fun(x + dx), dtype=float) - f0) / eps
    return jac


def ekf_predict(x: np.ndarray, p: np.ndarray, u: np.ndarray,
                process_model, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """EKF prediction step for a discrete-time nonlinear system."""
    f = lambda xx: process_model(xx, u)
    x_pred = np.asarray(f(x), dtype=float)
    F = _finite_difference_jacobian(f, x)
    p_pred = F @ p @ F.T + q
    return x_pred, p_pred


def ekf_update(x_pred: np.ndarray, p_pred: np.ndarray, z: np.ndarray,
               measurement_model, r: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """EKF correction step using numerical linearization of h(x)."""
    h = lambda xx: measurement_model(xx)
    z_pred = np.asarray(h(x_pred), dtype=float)
    H = _finite_difference_jacobian(h, x_pred)

    y = z - z_pred
    S = H @ p_pred @ H.T + r
    K = p_pred @ H.T @ np.linalg.inv(S)

    x_upd = x_pred + K @ y
    I = np.eye(p_pred.shape[0])
    p_upd = (I - K @ H) @ p_pred
    return x_upd, p_upd
