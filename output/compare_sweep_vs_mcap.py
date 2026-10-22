#!/usr/bin/env python
"""Overlay the rss_wrench_sweep mass_3000kg prediction on the junette_0_3000kg_0 mcap log.

The two datasets are the same 5 s Chewie reach trajectory: `mass_3000kg/` is what
`scripts/rss_reach.py` predicted, `mcap_csvs/junette_0_3000kg_0/` is what came back
off the RSS slide. Signals are matched by physical quantity, not by file name:

    base_motion.csv               <-> rss_slide_platform_state.csv
    chewie_joints.csv             <-> chewie_joint_states.csv
    han_joints.csv                <-> han_joint_states.csv
    chewie_fts_combined_wrench.csv<-> rss_slide_chewie_dyn_input_wrench.csv
    han_fts_wrench.csv            <-> rss_slide_han_dyn_input_wrench.csv

Two conventions had to be resolved from the data rather than guessed:

* The mcap `dyn_input_wrench` topics correspond to the `arm_on_mount_local`
  columns of the sweep CSVs. All four sweep conventions were correlated against
  the log; `arm_on_mount_*` correlates positively (`mount_on_arm_*` is its
  negation) and `*_local` beats `*_world_aligned` on RMS, matching the topics'
  `frame_id` (`chewie_fts_combined`, `han_fts`).
* `rss_slide_platform_state` RPY is in degrees. Roll peaks at 5.3 there while the
  platform translates 7 cm; as radians that would be a 306 deg tumble. Sweep RPY
  (radians) is converted to degrees for the overlay.

Time is aligned on the Chewie elbow: t = 0 is the first sample where the logged
elbow leaves its start angle. No time scaling is applied, so the log's slightly
longer execution (5.16 s vs the commanded 5.00 s) stays visible.
"""

import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT_ROOT = "/home/jh73385/Research/croc_crabs/output"
SWEEP = os.path.join(OUT_ROOT, "rss_wrench_sweep", "mass_3000kg")
MCAP = os.path.join(OUT_ROOT, "mcap_csvs", "junette_0_3000kg_0")
DEST = os.path.join(OUT_ROOT, "comparisons", "junette_0_3000kg_0_vs_mass_3000kg")

# Reference categorical slots 1 and 2, in fixed order. Linestyle carries the same
# distinction so the pair never reads by color alone.
C_PRED, C_LOG = "#2a78d6", "#eb6834"
PRED_KW = dict(color=C_PRED, lw=2.0, ls="--", zorder=3)
LOG_KW = dict(color=C_LOG, lw=2.0, ls="-", zorder=2)
LBL_PRED = "predicted (rss_wrench_sweep / mass_3000kg)"
LBL_LOG = "logged (mcap / junette_0_3000kg_0)"

TMIN, TMAX = -0.25, 6.0


def load(path):
    with open(path) as fh:
        rd = csv.reader(fh)
        return next(rd), list(rd)


def col(hdr, rows, name):
    i = hdr.index(name)
    return np.array([float(r[i]) if r[i] not in ("", "nan") else np.nan for r in rows])


def joint_stream(path, joint0):
    """mcap /joint_states rows interleave the arm pair with the gripper pair."""
    hdr, rows = load(path)
    i0 = hdr.index("name.0")
    keep = [r for r in rows if r[i0] == joint0]
    t = col(hdr, keep, "log_time_ns") * 1e-9
    return t, col(hdr, keep, "position.0"), col(hdr, keep, "position.1")


def style(ax, ylabel, title):
    ax.set_title(title, fontsize=10, pad=6)
    ax.set_ylabel(ylabel, fontsize=9)
    ax.grid(True, lw=0.5, color="#d8d8d4", zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color("#a8a8a3")
    ax.tick_params(labelsize=8, color="#a8a8a3")
    ax.set_xlim(TMIN, TMAX)


def finish(fig, axes, suptitle, note, fname):
    for ax in axes[-min(len(axes), 3):]:
        ax.set_xlabel("time since trajectory start [s]", fontsize=9)
    fig.suptitle(suptitle, fontsize=12, y=0.985)
    handles = [
        plt.Line2D([], [], **PRED_KW, label=LBL_PRED),
        plt.Line2D([], [], **LOG_KW, label=LBL_LOG),
    ]
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, 0.042),
               ncol=2, frameon=False, fontsize=9)
    fig.text(0.5, 0.008, note, ha="center", fontsize=7.5, color="#52514e")
    fig.tight_layout(rect=[0, 0.105, 1, 0.97])
    path = os.path.join(DEST, fname)
    fig.savefig(path, dpi=150, facecolor="#fcfcfb")
    plt.close(fig)
    print("wrote", path)


def rms_peak(ts, pred, tl, logv):
    """Residual stats over the commanded window, log resampled onto sweep times."""
    li = np.interp(ts, tl, logv)
    d = li - pred
    return np.sqrt(np.mean(d**2)), np.abs(d).max(), np.abs(pred).max(), np.abs(li).max()


def main():
    os.makedirs(DEST, exist_ok=True)

    # --- time alignment: first Chewie elbow sample off its start angle ----------
    t_ch, ch_elbow, ch_lift = joint_stream(
        os.path.join(MCAP, "chewie_joint_states.csv"), "chewie_elbow_joint"
    )
    t0 = t_ch[np.argmax(np.abs(ch_elbow - ch_elbow[0]) > 1e-4)]
    t_ch -= t0
    print(f"t0 = {t0:.6f} (log_time_ns epoch seconds)")

    summary = []

    # --- Chewie / Han arm joints ----------------------------------------------
    for side, mcap_f, joint0, sweep_f in [
        ("Chewie", "chewie_joint_states.csv", "chewie_elbow_joint", "chewie_joints.csv"),
        ("Han", "han_joint_states.csv", "han_elbow_joint", "han_joints.csv"),
    ]:
        tl, elbow, lift = joint_stream(os.path.join(MCAP, mcap_f), joint0)
        tl -= t0
        hs, rs = load(os.path.join(SWEEP, sweep_f))
        ts = col(hs, rs, "time_s")

        fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
        for ax, name, logv in [
            (axes[0], "shoulder_lift", lift),
            (axes[1], "elbow", elbow),
        ]:
            pred = col(hs, rs, f"{name}_pos")
            ax.plot(ts, pred, **PRED_KW)
            ax.plot(tl, logv, **LOG_KW)
            style(ax, "position [rad]", f"{side} {name}_joint")
            summary.append((f"{side} {name} [rad]",) + rms_peak(ts, pred, tl, logv))

        finish(
            fig,
            list(axes),
            f"{side} arm joint positions — 3000 kg reach",
            "Only shoulder_lift and elbow are published on the mcap /joint_states "
            "topic; the sweep's other four arm joints have no logged counterpart.",
            f"cmp_{side.lower()}_joints.png",
        )

    # --- platform / base motion ------------------------------------------------
    hp, rp = load(os.path.join(MCAP, "rss_slide_platform_state.csv"))
    tp = col(hp, rp, "log_time_ns") * 1e-9 - t0
    hb, rb = load(os.path.join(SWEEP, "base_motion.csv"))
    tb = col(hb, rb, "time_s")

    fig, axes = plt.subplots(2, 3, figsize=(13, 6.4))
    axes = axes.ravel()
    panels = [
        ("x", "base_x", "m", 1.0),
        ("y", "base_y", "m", 1.0),
        ("z", "base_z", "m", 1.0),
        ("roll", "base_roll", "deg", np.rad2deg(1.0)),
        ("pitch", "base_pitch", "deg", np.rad2deg(1.0)),
        ("yaw", "base_yaw", "deg", np.rad2deg(1.0)),
    ]
    for ax, (mc_c, sw_c, unit, scale) in zip(axes, panels):
        pred = col(hb, rb, sw_c) * scale
        logv = col(hp, rp, mc_c)
        ax.plot(tb, pred, **PRED_KW)
        ax.plot(tp, logv, **LOG_KW)
        style(ax, f"[{unit}]", f"platform {mc_c}")
        summary.append((f"base {mc_c} [{unit}]",) + rms_peak(tb, pred, tp, logv))

    finish(
        fig,
        list(axes),
        "Platform base motion — 3000 kg reach",
        "Sweep RPY converted rad -> deg; rss_slide_platform_state RPY is in degrees "
        "(roll peaks at 5.3 there while the base translates only 7 cm).",
        "cmp_base_motion.png",
    )

    # --- FTS wrenches ----------------------------------------------------------
    for side, mcap_f, sweep_f, frame in [
        ("Chewie", "rss_slide_chewie_dyn_input_wrench.csv",
         "chewie_fts_combined_wrench.csv", "chewie_fts_combined"),
        ("Han", "rss_slide_han_dyn_input_wrench.csv",
         "han_fts_wrench.csv", "han_fts"),
    ]:
        hw, rw = load(os.path.join(MCAP, mcap_f))
        tw = col(hw, rw, "log_time_ns") * 1e-9 - t0
        hs, rs = load(os.path.join(SWEEP, sweep_f))
        ts = col(hs, rs, "time_s")

        fig, axes = plt.subplots(2, 3, figsize=(13, 6.4))
        axes = axes.ravel()
        comps = [("fx", "force.x", "N"), ("fy", "force.y", "N"), ("fz", "force.z", "N"),
                 ("tx", "torque.x", "N m"), ("ty", "torque.y", "N m"), ("tz", "torque.z", "N m")]
        for ax, (comp, mc_c, unit) in zip(axes, comps):
            pred = col(hs, rs, f"arm_on_mount_local_{comp}")
            logv = col(hw, rw, f"wrench.{mc_c}")
            ax.plot(ts, pred, **PRED_KW)
            ax.plot(tw, logv, **LOG_KW)
            style(ax, f"[{unit}]", f"{frame} {comp}")
            summary.append((f"{side} {comp} [{unit}]",) + rms_peak(ts, pred, tw, logv))

        finish(
            fig,
            list(axes),
            f"{side} mount wrench ({frame}) — 3000 kg reach",
            "Sweep column set `arm_on_mount_local` — the convention the mcap "
            "dyn_input_wrench topic matches (positive correlation, lowest RMS).",
            f"cmp_{side.lower()}_wrench.png",
        )

    # --- residual table --------------------------------------------------------
    csv_path = os.path.join(DEST, "residual_summary.csv")
    with open(csv_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["signal", "rms_residual", "max_abs_residual",
                    "peak_abs_predicted", "peak_abs_logged"])
        for row in summary:
            w.writerow([row[0]] + [f"{v:.6g}" for v in row[1:]])
    print("wrote", csv_path)

    print(f"\n{'signal':22s} {'rms':>10s} {'max|err|':>10s} {'peak pred':>10s} {'peak log':>10s}")
    for name, rms, mx, pp, pl in summary:
        print(f"{name:22s} {rms:10.4f} {mx:10.4f} {pp:10.4f} {pl:10.4f}")


if __name__ == "__main__":
    main()
