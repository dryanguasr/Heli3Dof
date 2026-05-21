import numpy as np

from heli3dof.estimation import ekf_predict, ekf_update


def test_ekf_linear_system_matches_kalman_step():
    A = np.array([[1.0, 0.1], [0.0, 1.0]])
    B = np.array([0.0, 0.1])
    H = np.array([[1.0, 0.0]])

    def f(x, u):
        return A @ x + B * u[0]

    def h(x):
        return H @ x

    x0 = np.array([0.0, 1.0])
    p0 = np.eye(2) * 0.2
    q = np.eye(2) * 0.01
    r = np.array([[0.05]])
    u = np.array([1.0])
    z = np.array([0.08])

    xp, pp = ekf_predict(x0, p0, u, f, q)
    xu, pu = ekf_update(xp, pp, z, h, r)

    assert xp.shape == (2,)
    assert pp.shape == (2, 2)
    assert xu.shape == (2,)
    assert pu.shape == (2, 2)
    assert np.all(np.linalg.eigvals(pu) >= -1e-10)
