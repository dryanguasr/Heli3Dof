from __future__ import annotations

import numpy as np


def _triangular_membership(value: float, center: float, width: float) -> float:
    """Triangular membership function in [0, 1]."""
    distance = abs(value - center)
    if distance >= width:
        return 0.0
    return 1.0 - distance / width


def fuzzy_pd_torque(error: float, error_rate: float, *,
                    error_scale: float = 1.0,
                    rate_scale: float = 1.0,
                    output_gain: float = 1.0) -> float:
    """Compute a smooth control action using a compact fuzzy PD rule base.

    The rule base uses 3 linguistic labels for each input:
    negative, zero and positive.
    """
    e = np.clip(error / error_scale, -1.0, 1.0)
    de = np.clip(error_rate / rate_scale, -1.0, 1.0)

    mu_e = {
        "N": _triangular_membership(e, -1.0, 1.0),
        "Z": _triangular_membership(e, 0.0, 1.0),
        "P": _triangular_membership(e, 1.0, 1.0),
    }
    mu_de = {
        "N": _triangular_membership(de, -1.0, 1.0),
        "Z": _triangular_membership(de, 0.0, 1.0),
        "P": _triangular_membership(de, 1.0, 1.0),
    }

    # consequents in normalized output domain [-1, 1]
    rule_output = {
        ("N", "N"): -1.0,
        ("N", "Z"): -0.7,
        ("N", "P"): -0.3,
        ("Z", "N"): -0.4,
        ("Z", "Z"): 0.0,
        ("Z", "P"): 0.4,
        ("P", "N"): 0.3,
        ("P", "Z"): 0.7,
        ("P", "P"): 1.0,
    }

    numerator = 0.0
    denominator = 0.0
    for le, me in mu_e.items():
        for lde, mde in mu_de.items():
            firing = me * mde
            numerator += firing * rule_output[(le, lde)]
            denominator += firing

    if denominator == 0.0:
        return 0.0
    return output_gain * (numerator / denominator)


def fuzzy_pitch_roll_controller(state: np.ndarray,
                                references: tuple[float, float],
                                *,
                                hover_force: float = 0.0,
                                max_delta_force: float = 1.5) -> np.ndarray:
    """Simple fuzzy controller that maps pitch/roll errors to motor forces.

    State order: [theta, theta_dot, phi, phi_dot, psi, psi_dot].
    """
    theta_ref, phi_ref = references
    theta, theta_dot, phi, phi_dot = state[0], state[1], state[2], state[3]

    diff_force = fuzzy_pd_torque(theta_ref - theta, -theta_dot, output_gain=max_delta_force)
    collective = fuzzy_pd_torque(phi_ref - phi, -phi_dot, output_gain=max_delta_force)

    u_front = hover_force + collective + diff_force
    u_back = hover_force + collective - diff_force
    return np.array([u_front, u_back], dtype=float)
