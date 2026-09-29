"""Least-squares identification of arm inertial parameters from F/T wrenches.

    w = Y(q, v, a) @ pi + b + noise      (b: constant sensor bias, optional)

- base_parameters(): which combinations of pi the data can determine (rank / conditioning).
- fit_ols():          plain least squares on the identifiable subspace (minimum-norm deviation
                      from the prior), for diagnostics.
- fit_consistent():   the estimate to use. Least squares with every body's 4x4 pseudo-inertia
                      constrained to be positive semidefinite (physically valid mass, COM and
                      inertia per link), pulled weakly toward the prior (the URDF) in directions
                      the data does not determine. Gives a full per-link parameter set that can
                      be written back into the URDF / KDL YAML.
"""
import cvxpy as cp
import numpy as np

# typical magnitudes used to scale the prior term, per parameter type
_SCALE = np.array([1.0, 0.1, 0.1, 0.1, 0.05, 0.01, 0.05, 0.01, 0.01, 0.05])


def weights(n, sigma_f, sigma_t):
    """Row weights for stacked 6-row wrench samples (whitens force vs torque noise)."""
    return np.tile(1.0 / np.array([sigma_f] * 3 + [sigma_t] * 3), n)


def base_parameters(Y, tol=1e-6):
    """SVD of the stacked regressor. Returns rank, condition number of the identifiable part,
    and the row-space basis V_id (pi components that the data determines are V_id.T @ pi)."""
    s, vt = np.linalg.svd(Y, full_matrices=False)[1:]
    r = int((s > tol * s[0]).sum())
    return r, s[0] / s[r - 1], vt[:r]


def _augment(Y, n, bias):
    return np.hstack([Y, np.tile(np.eye(6), (n, 1))]) if bias else Y


def fit_ols(Y, w, prior, W=None, bias=False):
    n = len(w) // 6
    A = _augment(Y, n, bias)
    W = np.ones(len(w)) if W is None else W
    x0 = np.concatenate([prior, np.zeros(6)]) if bias else prior
    dx = np.linalg.lstsq(A * W[:, None], (w - A @ x0) * W, rcond=None)[0]
    x = x0 + dx
    return (x[:-6], x[-6:]) if bias else (x, np.zeros(6))


def fit_consistent(model, Y, w, prior, W=None, bias=False, lam=1e-3, eps=1e-6):
    """model: ArmFT (for pseudo_inertia). lam weights the pull toward the prior relative to
    the (whitened, per-sample averaged) data misfit."""
    n = len(w) // 6
    A = _augment(Y, n, bias)
    W = np.ones(len(w)) if W is None else W
    Aw, bw = A * W[:, None], w * W
    # reduce the data term to a small square form: ||R x - z||^2 == ||Aw x - bw||^2 + const
    Q, R = np.linalg.qr(Aw)
    z = Q.T @ bw
    nb = model.nb
    x = cp.Variable(A.shape[1])
    pi = x[:10 * nb]
    scale = np.tile(_SCALE, nb)
    cost = cp.sum_squares(R @ x - z) / n + lam * cp.sum_squares(cp.multiply(1 / scale, pi - prior))
    cons = []
    for k in range(nb):
        p = pi[10 * k:10 * k + 10]
        m, hx, hy, hz, ixx, ixy, iyy, ixz, iyz, izz = [p[i] for i in range(10)]
        tr = 0.5 * (ixx + iyy + izz)
        J = cp.bmat([[tr - ixx, -ixy, -ixz, hx],
                     [-ixy, tr - iyy, -iyz, hy],
                     [-ixz, -iyz, tr - izz, hz],
                     [hx, hy, hz, m]])
        cons.append(J >> eps * np.eye(4))
    prob = cp.Problem(cp.Minimize(cost), cons)
    prob.solve(solver=cp.CLARABEL)
    if prob.status not in ("optimal", "optimal_inaccurate"):
        raise RuntimeError(f"identification failed: {prob.status}")
    xv = x.value
    return (xv[:10 * nb], xv[10 * nb:]) if bias else (xv, np.zeros(6))


def body_summary(pi):
    """Per body: mass, COM (body frame) and inertia about the COM, from Pinocchio dynamic params."""
    import pinocchio as pin
    out = []
    for k in range(len(pi) // 10):
        I = pin.Inertia.FromDynamicParameters(pi[10 * k:10 * k + 10])
        out.append(dict(m=I.mass, c=I.lever.copy(), I=I.inertia.copy()))
    return out
