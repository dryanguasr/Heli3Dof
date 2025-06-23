import numpy as np

def rot_x(a: float) -> np.ndarray:
    """Rotation matrix around X-axis."""
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])


def rot_y(a: float) -> np.ndarray:
    """Rotation matrix around Y-axis."""
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[ca, 0, sa], [0, 1, 0], [-sa, 0, ca]])


def rot_z(a: float) -> np.ndarray:
    """Rotation matrix around Z-axis."""
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[ca, -sa, 0], [sa, ca, 0], [0, 0, 1]])
