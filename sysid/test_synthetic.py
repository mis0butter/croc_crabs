"""End-to-end test of the identification pipeline on simulated data with known parameters.

"Truth" = URDF + things the URDF leaves out (camera on wrist_3, cable harnesses on the
upper arm and forearm, extra mass on the sensor stack) + a few % random error per link.
We simulate what the hardware logs (joint states at 500 Hz, F/T at 250 Hz with noise and an
unknown constant bias, platform pose at 250 Hz), run the same preprocessing and fit that
real data will go through, and check the recovered parameters and the wrench prediction on
a held-out trajectory.

    python sysid/test_synthetic.py [--arm chewie] [--plot out.png]
"""
import argparse
import os
import sys

import numpy as np
import pinocchio as pin

sys.path.insert(0, os.path.dirname(__file__))
from arm_model import ArmFT, FT_LINK  # noqa: E402
from identify import base_parameters, body_summary, fit_consistent, fit_ols, weights  # noqa: E402
from preprocess import derivatives, lowpass, resample  # noqa: E402

URDF = os.path.join(os.path.dirname(__file__), "..", "rss_platform", "sim_chaser.urdf")
FS_Q, FS_W = 500.0, 250.0
SIG_Q, SIG_F, SIG_T = 2e-5, 0.5, 0.05          # encoder [rad], F/T noise [N], [N m]
HOME = {"han": [0.0, 0.0, -2.6, 0.0, 0.0, 0.0], "chewie": [3.14, 3.14, 2.6, 0.0, 0.0, 0.0]}


# ------------------------------------------------------------------ ground truth
def point(m, p, size=0.05):
    return pin.Inertia.FromBox(m, size, size, size).se3Action(pin.SE3(np.eye(3), np.array(p, float)))


def true_params(A, rng):
    """URDF + unmodelled hardware + ~5% random error per body (all physically consistent)."""
    add = {0: point(0.3, [0.05, 0.0, 0.10]),        # sensor stack: bolts / cabling
           2: point(0.9, [-0.40, 0.0, 0.18]),        # upper arm: cable harness
           3: point(0.5, [-0.35, 0.0, 0.05]),        # forearm: cable harness
           6: point(0.4, [0.0, 0.07, 0.09])}         # wrist_3: camera + mount
    pis = []
    for k in range(A.nb):
        I = A.model.inertias[k + 1]
        s = 1 + rng.uniform(-0.05, 0.05)
        I = pin.Inertia(I.mass * s, I.lever + rng.normal(0, 0.005, 3), I.inertia * s)
        if k in add:
            I = I + add[k]
        pis.append(I.toDynamicParameters())
    return np.concatenate(pis)


# ------------------------------------------------------------------ trajectories
def fourier(rng, q0, T, fs, n_harm=5, f0=0.1, vmax=0.8):
    """Periodic excitation q(t) = q0 + sum_k a_k sin(k w t) + b_k cos(k w t) (analytic derivatives)."""
    t = np.arange(0, T, 1 / fs)
    w = 2 * np.pi * f0 * np.arange(1, n_harm + 1)
    a, b = rng.normal(size=(2, 6, n_harm)) / np.arange(1, n_harm + 1)
    S, C = np.sin(np.outer(t, w)), np.cos(np.outer(t, w))
    q = (S @ a.T + C @ b.T)
    v = (C * w) @ a.T - (S * w) @ b.T
    acc = -(S * w**2) @ a.T - (C * w**2) @ b.T
    k = vmax / np.abs(v).max(axis=0)
    q, v, acc = q * k, v * k, acc * k
    return t, q0 + q - q[0], v, acc          # start at q0


def static_poses(rng, q0, n, hold, fs, spread=1.0):
    """n random poses, each held for `hold` s (smooth 1.5 s moves between them)."""
    poses = [np.array(q0)] + [q0 + rng.uniform(-spread, spread, 6) for _ in range(n)]
    seg = []
    for p0, p1 in zip(poses[:-1], poses[1:]):
        s = np.linspace(0, 1, int(1.5 * fs))
        blend = 10 * s**3 - 15 * s**4 + 6 * s**5
        seg.append(p0 + np.outer(blend, p1 - p0))
        seg.append(np.tile(p1, (int(hold * fs), 1)))
    q = np.vstack(seg)
    t = np.arange(len(q)) / fs
    dt = 1 / fs
    v = np.gradient(q, dt, axis=0)
    return t, q, v, np.gradient(v, dt, axis=0)


def platform(rng, t, tilt_deg=5.0):
    """Slow platform tilt + small translation, like the hexapod. Returns R_world_plate(t),
    plate angular velocity / acceleration (world) and linear acceleration of the plate origin."""
    ph = rng.uniform(0, 2 * np.pi, 3)
    f = np.array([0.03, 0.041, 0.023])
    th = np.radians([tilt_deg, tilt_deg, 2.0])[None] * np.sin(2 * np.pi * f[None] * t[:, None] + ph)
    R = np.array([pin.rpy.rpyToMatrix(*x) for x in th])
    dt = t[1] - t[0]
    om = np.array([pin.log3(R[i + 1] @ R[i].T) / dt for i in range(len(t) - 1)])
    om = np.vstack([om, om[-1]])
    al = np.gradient(om, dt, axis=0)
    acc = 0.05 * np.sin(2 * np.pi * np.array([0.05, 0.07, 0.03])[None] * t[:, None] + ph)
    return R, om, al, acc


def plate_to_ft(arm):
    full = pin.buildModelFromUrdf(URDF)
    d = full.createData()
    pin.framesForwardKinematics(full, d, pin.neutral(full))
    return d.oMf[full.getFrameId(FT_LINK[arm])]       # plate = world at neutral


# ------------------------------------------------------------------ simulate + measure
def ft_states(A, X_pf, qj, vj, aj, R_wp, om_w, al_w, acc_w):
    """Per-sample (q, v, a) of the F/T-rooted model from joint + platform motion."""
    out = []
    p = X_pf.translation
    for i in range(len(qj)):
        R_wf = R_wp[i] @ X_pf.rotation
        Rfw = R_wf.T
        om, al = om_w[i], al_w[i]
        r = R_wp[i] @ p                                  # plate origin -> sensor, world
        a_pt = acc_w[i] + np.cross(al, r) + np.cross(om, np.cross(om, r))
        out.append(A.state(R_wf, qj[i], omega=Rfw @ om, dv=Rfw @ a_pt, alpha=Rfw @ al, vj=vj[i], aj=aj[i]))
    return out


def simulate(A, pi_true, rng, t, qj, vj, aj, bias):
    X_pf = plate_to_ft(A.arm)
    R_wp, om, al, acc = platform(rng, t)
    A.set_params(pi_true)
    w_true = np.array([A.wrench(*s) for s in ft_states(A, X_pf, qj, vj, aj, R_wp, om, al, acc)])
    A.set_params(A.pi_urdf)
    # what the loggers would record
    t_w = np.arange(t[0], t[-1], 1 / FS_W)
    w_meas = resample(t, w_true, t_w) + bias + rng.normal(0, 1, (len(t_w), 6)) * np.array([SIG_F] * 3 + [SIG_T] * 3)
    rpy = np.array([pin.rpy.matrixToRpy(R) for R in R_wp])
    pos = np.cumsum(np.cumsum(acc, axis=0), axis=0) / FS_Q**2          # plate position (double integral)
    return dict(t=t, q=qj + rng.normal(0, SIG_Q, qj.shape), t_w=t_w, w=w_meas,
                platform=np.hstack([pos, rpy]), w_true=w_true)


def rows(A, run, fc=15.0, still_only=False):
    """The preprocessing real data will get: SG derivatives of q and platform pose, low-pass
    F/T, everything on the F/T timestamps. Returns stacked Y, w."""
    q, v, a = derivatives(run["q"], FS_Q)
    pos, vel, acc = derivatives(run["platform"][:, :3], FS_Q)
    ang = run["platform"][:, 3:]
    R_wp = np.array([pin.rpy.rpyToMatrix(*x) for x in ang])
    om = np.array([pin.log3(R_wp[min(i + 1, len(R_wp) - 1)] @ R_wp[i].T) * FS_Q for i in range(len(R_wp))])
    om = derivatives(om, FS_Q)[0]
    al = derivatives(om, FS_Q)[1]
    t_w = run["t_w"]
    idx = np.clip(np.searchsorted(run["t"], t_w), 0, len(run["t"]) - 1)   # nearest joint sample
    X_pf = plate_to_ft(A.arm)
    states = ft_states(A, X_pf, q[idx], v[idx], a[idx], R_wp[idx], om[idx], al[idx], acc[idx])
    Y = np.vstack([A.regressor(*s) for s in states])
    w = lowpass(run["w"], FS_W, fc)
    if still_only:   # Stage A: keep only samples where the arm is at rest
        keep = np.abs(v[idx]).max(axis=1) < 1e-3
        keep &= np.convolve(keep, np.ones(int(0.3 * FS_W)), "same") >= int(0.3 * FS_W)   # 0.15 s margin
        Y, w = Y.reshape(-1, 6, Y.shape[1])[keep].reshape(-1, Y.shape[1]), w[keep]
    return Y, w.ravel(), states


def rms(A, pi, bias, Y, w):
    e = (Y @ pi + np.tile(bias, len(w) // 6) - w).reshape(-1, 6)
    return np.sqrt((e**2).mean(axis=0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="chewie", choices=["han", "chewie"])
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--plot")
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    A = ArmFT(URDF, args.arm)
    pi_true = true_params(A, rng)
    bias = np.concatenate([rng.normal(0, 3, 3), rng.normal(0, 0.3, 3)])      # unknown sensor offset
    q0 = np.array(HOME[args.arm])
    q0 = q0 + np.array([-1.5, 0.0, -1.0, 0.0, 0.0, 0.0]) * (1 if args.arm == "chewie" else 0)

    runs = {"static": static_poses(rng, q0, 30, 2.0, FS_Q),
            "excite": fourier(rng, q0, 30.0, FS_Q),
            "holdout": fourier(rng, q0, 20.0, FS_Q)}
    data = {}
    for k, (t, q, v, a) in runs.items():
        run = simulate(A, pi_true, rng, t, q, v, a, bias)
        Y, w, _ = rows(A, run, still_only=(k == "static"))
        data[k] = (Y, w)
        print(f"{k:8s}: {len(t) / FS_Q:5.1f} s, {len(w) // 6} wrench samples used")

    W = lambda w: weights(len(w) // 6, SIG_F, SIG_T)          # noqa: E731
    Yh, wh = data["holdout"]

    print("\nidentifiability (whitened):")
    for name, keys in [("static only", ["static"]), ("static + excitation", ["static", "excite"])]:
        Y = np.vstack([data[k][0] for k in keys]); w = np.concatenate([data[k][1] for k in keys])
        r, cond, _ = base_parameters(Y * W(w)[:, None])
        print(f"  {name:22s} rank {r:2d} / {Y.shape[1]}  cond {cond:9.1f}")

    print("\nheld-out wrench RMS  [Fx Fy Fz] N | [Tx Ty Tz] N·m")
    b_urdf = fit_ols(np.zeros((len(wh), 0)), wh - Yh @ A.pi_urdf, np.zeros(0), bias=True)[1]  # URDF + tare
    show = lambda name, e: print(f"  {name:32s} " + " ".join(f"{x:6.2f}" for x in e[:3]) + " | " + " ".join(f"{x:6.3f}" for x in e[3:]))  # noqa: E731
    show("truth (noise floor)", rms(A, pi_true, bias, Yh, wh))
    show("URDF (sensor tared on held-out)", rms(A, A.pi_urdf, b_urdf, Yh, wh))
    fits = {}
    for name, keys in [("static only", ["static"]), ("static + excitation", ["static", "excite"])]:
        Y = np.vstack([data[k][0] for k in keys]); w = np.concatenate([data[k][1] for k in keys])
        fits[name] = fit_consistent(A, Y, w, A.pi_urdf, W=W(w), bias=True)
        show(f"identified: {name}", rms(A, *fits[name], Yh, wh))

    print("\nper-body mass [kg] and COM error [mm]     URDF  ->  identified (static+exc)  vs  truth")
    S_u, S_t, S_i = body_summary(A.pi_urdf), body_summary(pi_true), body_summary(fits["static + excitation"][0])
    for k in range(A.nb):
        cu = 1e3 * np.linalg.norm(S_u[k]["c"] - S_t[k]["c"]); ci = 1e3 * np.linalg.norm(S_i[k]["c"] - S_t[k]["c"])
        print(f"  B{k} {A.body_names[k]:32s} m {S_u[k]['m']:7.3f} -> {S_i[k]['m']:7.3f}  (true {S_t[k]['m']:7.3f})"
              f"   COM err {cu:5.1f} -> {ci:5.1f}")
    print(f"  sensor bias estimate error: {np.round(fits['static + excitation'][1] - bias, 3)}")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
        tt = np.arange(len(wh) // 6) / FS_W
        pi_i, b_i = fits["static + excitation"]
        for i, lab in enumerate(["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]):
            ax[i].plot(tt, wh.reshape(-1, 6)[:, i], "k", lw=0.6, label="measured")
            ax[i].plot(tt, (Yh @ A.pi_urdf).reshape(-1, 6)[:, i] + b_urdf[i], "C1", lw=1, label="URDF")
            ax[i].plot(tt, (Yh @ pi_i).reshape(-1, 6)[:, i] + b_i[i], "C0--", lw=1, label="identified")
            ax[i].set_ylabel(lab)
        ax[0].legend(ncol=3)
        ax[-1].set_xlabel("t [s] (held-out trajectory)")
        plt.tight_layout()
        plt.savefig(args.plot, dpi=80)
        print(f"wrote {args.plot}")


if __name__ == "__main__":
    main()
