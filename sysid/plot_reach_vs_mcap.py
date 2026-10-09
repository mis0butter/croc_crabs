"""Plot a hardware log against the rss_reach.py prediction for the same base mass.

Inputs:
  the .npz written by extract_mcap.py
  the output directory of `scripts/rss_reach.py plot --mass-kg <kg> --out-dir <dir>`

The prediction starts at t = 0; it is shifted onto the log by the offset that best matches
the chewie joint positions. Three figures are written to --out:
  joints.png   joint positions and velocities, both arms
  base.png     platform pose (/rss/slide_platform_state) vs the simulated base
  wrench.png   *_dyn_input_wrench vs the simulated sensor wrench

The two wrenches are NOT the same quantity: the log holds F/T minus the KDL arm prediction
(tared), the simulation holds the full zero-gravity wrench at the sensor.

  python sysid/plot_reach_vs_mcap.py <run>.npz <reach_out_dir> --out <dir>
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
WRENCH = ["fx", "fy", "fz", "tx", "ty", "tz"]
SENSOR = {"chewie": "chewie_fts_combined", "han": "han_fts"}
SIGNS = ("arm_on_mount", "mount_on_arm")
MEAS = dict(color="0.15", lw=1.2, label="measured")
PRED = dict(color="tab:red", lw=1.4, label="rss_reach.py")


def read_csv(path):
    with open(path) as f:
        cols = f.readline().strip().split(",")
    return dict(zip(cols, np.loadtxt(path, delimiter=",", skiprows=1).T))


def hold(t, tp, yp):
    """Prediction on the log's time base, held at its first/last value outside the horizon."""
    return np.interp(t, tp, yp)


def find_offset(t, q, pred):
    """Log time of the prediction's t = 0, from the chewie joint positions."""
    tp = pred["time_s"]
    qp = np.stack([pred[f"{j}_pos"] for j in JOINTS], axis=1)
    best = (np.inf, 0.0)
    for off in np.arange(t[0], t[-1] - tp[-1], 0.01):
        err = np.stack([hold(t - off, tp, qp[:, i]) for i in range(6)], axis=1) - q
        best = min(best, (np.sqrt((err**2).mean()), off))
    return best[1], best[0]


def finish(fig, axes, xlim, path):
    for ax in axes.flat:
        ax.grid(True, alpha=0.3)
        ax.set_xlim(*xlim)
        ax.tick_params(labelsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("log time [s]")
    axes[0, 0].legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("wrote", path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("reach_dir")
    ap.add_argument("--out", default=".")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    d = np.load(args.npz, allow_pickle=True)
    t0 = float(d["t0"])
    run = os.path.splitext(os.path.basename(args.npz))[0]
    joints = {arm: read_csv(os.path.join(args.reach_dir, f"{arm}_joints.csv")) for arm in SENSOR}
    base = read_csv(os.path.join(args.reach_dir, "base_motion.csv"))
    wrench = {arm: read_csv(os.path.join(args.reach_dir, f"{s}_wrench.csv")) for arm, s in SENSOR.items()}

    off, rms = find_offset(d["chewie_t"] - t0, d["chewie_q"], joints["chewie"])
    horizon = base["time_s"][-1]
    t_end = max(d[f"{k}_t"][-1] for k in ("chewie", "platform")) - t0
    xlim = (off - 1.5, t_end)
    print(f"prediction t=0 at log t={off:.2f} s (chewie joint RMS {rms:.4f} rad)")

    # ---- joints
    fig, axes = plt.subplots(4, 6, figsize=(20, 11), sharex=True)
    for r, arm in enumerate(SENSOR):
        t = d[f"{arm}_t"] - t0
        for c, j in enumerate(JOINTS):
            for k, (kind, meas, unit) in enumerate((("pos", d[f"{arm}_q"], "rad"), ("vel", d[f"{arm}_v"], "rad/s"))):
                ax = axes[2 * r + k, c]
                ax.plot(t, meas[:, c], **MEAS)
                ax.plot(joints[arm]["time_s"] + off, joints[arm][f"{j}_{kind}"], **PRED)
                ax.set_title(f"{arm} {j} {kind} [{unit}]", fontsize=9)
            err = hold(t - off, joints[arm]["time_s"], joints[arm][f"{j}_pos"]) - d[f"{arm}_q"][:, c]
            sel = (t >= off) & (t <= off + horizon)
            print(f"  {arm:6s} {j:13s} pos error: rms {np.sqrt((err[sel]**2).mean()):.4f}  max {np.abs(err[sel]).max():.4f} rad")
    fig.suptitle(f"{run}: joints, measured vs rss_reach.py", fontweight="bold")
    finish(fig, axes, (off - 1.5, off + horizon + 3.0), os.path.join(args.out, "joints.png"))

    # ---- base
    tp, p = d["platform_t"] - t0, d["platform"]
    p = p - p[tp < off].mean(axis=0)
    fig, axes = plt.subplots(2, 3, figsize=(15, 7), sharex=True)
    for c, name in enumerate(["x", "y", "z", "roll", "pitch", "yaw"]):
        ax = axes.flat[c]
        scale, unit = (1e3, "mm") if c < 3 else (1.0, "deg")
        pred = base[f"base_{name}"] * (1e3 if c < 3 else np.degrees(1.0))
        ax.plot(tp, p[:, c] * scale, **MEAS)
        ax.plot(base["time_s"] + off, pred, **PRED)
        ax.axvline(off + horizon, color="0.6", ls=":", lw=1)
        ax.set_title(f"base {name} [{unit}]", fontsize=10)
        end = p[np.argmin(np.abs(tp - off - horizon)), c] * scale
        print(f"  base {name:5s} at end of horizon: measured {end:8.2f}  predicted {pred[-1]:8.2f} {unit}")
    fig.suptitle(f"{run}: platform state vs simulated base (measured rpy assumed degrees; dotted = end of horizon)",
                 fontweight="bold")
    finish(fig, axes, xlim, os.path.join(args.out, "base.png"))

    # ---- wrench
    fig, axes = plt.subplots(2, 6, figsize=(20, 6.5), sharex=True)
    for r, (arm, sensor) in enumerate(SENSOR.items()):
        t, w = d[f"{arm}_wrench_t"] - t0, d[f"{arm}_wrench"]
        sel = (t >= off) & (t <= off + horizon)
        pw = {s: np.stack([wrench[arm][f"{s}_local_{c}"] for c in WRENCH], axis=1) for s in SIGNS}
        corr = np.array([np.corrcoef(w[sel, c], hold(t[sel] - off, wrench[arm]["time_s"], pw[SIGNS[0]][:, c]))[0, 1]
                         for c in range(6)])
        print(f"  {arm:6s} wrench correlation with {SIGNS[0]}: " + " ".join(f"{c}={v:+.2f}" for c, v in zip(WRENCH, corr)))
        for c, name in enumerate(WRENCH):
            ax = axes[r, c]
            ax.plot(t, w[:, c], **MEAS)
            ax.plot(wrench[arm]["time_s"] + off, pw[SIGNS[0]][:, c], **PRED)
            ax.set_title(f"{sensor} {name} [{'N' if c < 3 else 'N m'}]  r={corr[c]:+.2f}", fontsize=9)
    fig.suptitle(f"{run}: *_dyn_input_wrench (F/T minus KDL arm model, tared) vs simulated zero-g sensor wrench "
                 f"({SIGNS[0]}, sensor frame)", fontweight="bold")
    finish(fig, axes, xlim, os.path.join(args.out, "wrench.png"))


if __name__ == "__main__":
    main()
