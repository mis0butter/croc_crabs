# ref. https://github.com/PyCQA/pycodestyle/issues/373, remove this for ruff
import os
import signal
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import pinocchio
from pinocchio.visualize import MeshcatVisualizer
from pinocchio import JointModelFreeFlyer

import crocoddyl

# ====================================================================
# Loading the RSS chaser robot model
# ====================================================================

# argument handling
WITHDISPLAY = "display" in sys.argv or "CROCODDYL_DISPLAY" in os.environ
WITHVIDEO = "video" in sys.argv
WITHPLOT = "plot" in sys.argv or "CROCODDYL_PLOT" in os.environ
signal.signal(signal.SIGINT, signal.SIG_DFL)

# Define paths
model_dir = os.path.join(os.path.dirname(__file__), "..", "rss_platform")
urdf_path = os.path.join(model_dir, "sim_chaser.urdf")
package_dirs = [model_dir]

# Pinocchio adds a free flyer at the root. The URDF also has a floating
# slide_optical_plate joint; fix it here so actuation applies to the arms only.
urdf_text = open(urdf_path).read()
urdf_text = urdf_text.replace(
    'name="slide_optical_plate_joint" type="floating"',
    'name="slide_optical_plate_joint" type="fixed"',
)
with tempfile.NamedTemporaryFile(suffix=".urdf", mode="w", delete=False) as tmp:
    tmp.write(urdf_text)
    urdf_load_path = tmp.name

# Load model + visuals
model, collision_model, visual_model = pinocchio.buildModelsFromUrdf(
    urdf_load_path, package_dirs, JointModelFreeFlyer()
)

# turn off gravity!
model.gravity.linear[:] = 0.0

# Create data, state, actuation models
data = model.createData()
state = crocoddyl.StateMultibody(model)
# Root free flyer is unactuated; arm joints are actuated.
actuation = crocoddyl.ActuationModelFloatingBase(state)

# ====================================================================
# Initial arm pose [rad]
# ====================================================================

ARM_INIT_POS = {
    # Han: tucked pose for base stabilization
    "han_shoulder_pan_joint": 0.0,
    "han_shoulder_lift_joint": 0.0,
    "han_elbow_joint": -2.6,
    "han_wrist_1_joint": 0.0,
    "han_wrist_2_joint": 0.0,
    "han_wrist_3_joint": 0.0,
    "han_gripper_finger1_joint": 0.0,
    "han_gripper_finger2_joint": 0.0,
    # Chewie: reaching pose
    "chewie_shoulder_pan_joint": 3.14,
    "chewie_shoulder_lift_joint": 3.14,
    "chewie_elbow_joint": 2.6,
    "chewie_wrist_1_joint": 0.0,
    "chewie_wrist_2_joint": 0.0,
    "chewie_wrist_3_joint": 0.0,
    "chewie_gripper_finger1_joint": 0.0,
    "chewie_gripper_finger2_joint": 0.0,
}

q0 = pinocchio.neutral(model)
for joint_name, angle in ARM_INIT_POS.items():
    q0[model.joints[model.getJointId(joint_name)].idx_q] = angle

HAN_JOINTS = [name for name in ARM_INIT_POS if name.startswith("han_")]
CHEWIE_JOINTS = [name for name in ARM_INIT_POS if name.startswith("chewie_")]


def _joint_label(joint_name):
    return joint_name.replace("han_", "").replace("chewie_", "").replace("_joint", "")


def _make_output_dir():
    stamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    out_dir = os.path.join(os.path.dirname(__file__), "..", "output", stamp)
    os.makedirs(out_dir, exist_ok=True)
    return out_dir


def _write_csv(path, columns, rows):
    header = ",".join(columns)
    with open(path, "w") as f:
        f.write(header + "\n")
        for row in rows:
            f.write(",".join(f"{val:.9g}" for val in row) + "\n")


def _extract_trajectory_data(xs, model, data, dt, ee_frame_id, ee_goal):
    times = np.arange(len(xs)) * dt
    positions = {name: [] for name in ARM_INIT_POS}
    velocities = {name: [] for name in ARM_INIT_POS}
    ee_errors = []
    ee_pos = []
    base_rpy = []
    base_omega = []
    com_pos = []

    for x in xs:
        q = x[: model.nq]
        v = x[model.nq :]
        for joint_name in ARM_INIT_POS:
            jid = model.getJointId(joint_name)
            positions[joint_name].append(q[model.joints[jid].idx_q])
            velocities[joint_name].append(v[model.joints[jid].idx_v])

        pinocchio.forwardKinematics(model, data, q)
        pinocchio.centerOfMass(model, data, q)
        pinocchio.updateFramePlacement(model, data, ee_frame_id)
        ee_position = data.oMf[ee_frame_id].translation.copy()
        ee_errors.append(np.linalg.norm(ee_position - ee_goal))
        ee_pos.append(ee_position)

        quat = pinocchio.Quaternion(q[6], q[3], q[4], q[5])
        base_rpy.append(pinocchio.rpy.matrixToRpy(quat.toRotationMatrix()))
        base_omega.append(v[3:6])
        com_pos.append(data.com[0].copy())

    return {
        "times": times,
        "positions": {name: np.asarray(vals) for name, vals in positions.items()},
        "velocities": {name: np.asarray(vals) for name, vals in velocities.items()},
        "ee_errors": np.asarray(ee_errors),
        "ee_pos": np.asarray(ee_pos),
        "base_rpy": np.asarray(base_rpy),
        "base_omega": np.asarray(base_omega),
        "com_pos": np.asarray(com_pos),
    }


def parse_urdf_joint_limits(urdf_path, joint_names):
    limits = {}
    for joint in ET.parse(urdf_path).getroot().findall("joint"):
        name = joint.get("name")
        if name not in joint_names:
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        limits[name] = {
            "lower": float(limit.get("lower")),
            "upper": float(limit.get("upper")),
            "velocity": float(limit.get("velocity")),
        }
    return limits


def _plot_joint_limits(ax, joint_name, limits, kind):
    if joint_name not in limits:
        return
    if kind == "position":
        ax.axhline(
            limits[joint_name]["lower"],
            color="gray",
            ls="--",
            lw=1,
            alpha=0.8,
            label="limit",
        )
        ax.axhline(
            limits[joint_name]["upper"],
            color="gray",
            ls="--",
            lw=1,
            alpha=0.8,
        )
    else:
        vlim = limits[joint_name]["velocity"]
        ax.axhline(vlim, color="gray", ls="--", lw=1, alpha=0.8, label="limit")
        ax.axhline(-vlim, color="gray", ls="--", lw=1, alpha=0.8)


def _save_arm_joint_figure(out_dir, traj, limits, arm_name, joint_names, color, linestyle):
    times = traj["times"]
    positions = traj["positions"]
    velocities = traj["velocities"]

    fig = plt.figure(figsize=(14, 11))
    gs = fig.add_gridspec(
        5,
        4,
        height_ratios=[1, 1, 0.1, 1, 1],
        hspace=0.4,
        wspace=0.25,
        top=0.93,
        bottom=0.06,
        left=0.06,
        right=0.98,
    )

    csv_columns = ["time_s"]
    csv_rows = []
    for k, time_s in enumerate(times):
        row = [time_s]
        for joint_name in joint_names:
            row.extend([positions[joint_name][k], velocities[joint_name][k]])
        csv_rows.append(row)

    for joint_idx, joint_name in enumerate(joint_names):
        label = _joint_label(joint_name)
        csv_columns.extend([f"{label}_pos", f"{label}_vel"])

        pos_row, pos_col = divmod(joint_idx, 4)
        pos_ax = fig.add_subplot(gs[pos_row, pos_col])
        vel_ax = fig.add_subplot(gs[pos_row + 3, pos_col], sharex=pos_ax)

        pos_ax.plot(times, positions[joint_name], color=color, ls=linestyle, label=arm_name)
        _plot_joint_limits(pos_ax, joint_name, limits, "position")
        pos_ax.set_title(label)
        pos_ax.grid(True, alpha=0.3)

        vel_ax.plot(times, velocities[joint_name], color=color, ls=linestyle)
        _plot_joint_limits(vel_ax, joint_name, limits, "velocity")
        vel_ax.set_title(label)
        vel_ax.grid(True, alpha=0.3)

        if joint_idx == 0:
            pos_ax.legend(loc="upper right", fontsize=7)
        if pos_row == 0:
            plt.setp(pos_ax.get_xticklabels(), visible=False)
        else:
            pos_ax.set_xlabel("time [s]")
        if pos_row == 0:
            plt.setp(vel_ax.get_xticklabels(), visible=False)
        else:
            vel_ax.set_xlabel("time [s]")

    fig.text(
        0.5,
        0.975,
        f"{arm_name} joint positions [rad]",
        ha="center",
        fontsize=15,
        fontweight="bold",
    )
    fig.text(
        0.5,
        0.48,
        f"{arm_name} joint velocities [rad/s]",
        ha="center",
        fontsize=15,
        fontweight="bold",
    )
    fig.savefig(
        os.path.join(out_dir, f"{arm_name}_joints.png"), dpi=150, bbox_inches="tight"
    )
    _write_csv(os.path.join(out_dir, f"{arm_name}_joints.csv"), csv_columns, csv_rows)
    plt.close(fig)


def _save_figure_b(out_dir, traj, q_ref, ee_goal):
    times = traj["times"]
    rpy_ref = pinocchio.rpy.matrixToRpy(
        pinocchio.Quaternion(q_ref[6], q_ref[3], q_ref[4], q_ref[5]).toRotationMatrix()
    )

    csv_columns = [
        "time_s",
        "base_roll",
        "base_pitch",
        "base_yaw",
        "base_wx",
        "base_wy",
        "base_wz",
        "com_x",
        "com_y",
        "com_z",
        "ee_x",
        "ee_y",
        "ee_z",
        "ee_error",
    ]
    csv_rows = [
        [
            t,
            *traj["base_rpy"][k],
            *traj["base_omega"][k],
            *traj["com_pos"][k],
            *traj["ee_pos"][k],
            traj["ee_errors"][k],
        ]
        for k, t in enumerate(times)
    ]

    fig = plt.figure(figsize=(12, 9))
    ax_ori = fig.add_subplot(2, 2, 1)
    ax_3d = fig.add_subplot(2, 2, 2, projection="3d")
    ax_ang = fig.add_subplot(2, 2, 3)
    ax_err = fig.add_subplot(2, 2, 4)

    rpy_labels = ["roll", "pitch", "yaw"]
    for i, label in enumerate(rpy_labels):
        ax_ori.plot(times, traj["base_rpy"][:, i], label=label)
        ax_ori.axhline(rpy_ref[i], color=f"C{i}", ls=":", lw=1, alpha=0.7)
    ax_ori.set_xlabel("time [s]")
    ax_ori.set_ylabel("orientation [rad]")
    ax_ori.set_title("Base orientation (RPY)")
    ax_ori.grid(True, alpha=0.3)
    ax_ori.legend(ncol=3, fontsize=8)

    omega_labels = ["wx", "wy", "wz"]
    for i, label in enumerate(omega_labels):
        ax_ang.plot(times, traj["base_omega"][:, i], label=label)
    ax_ang.set_xlabel("time [s]")
    ax_ang.set_ylabel("angular velocity [rad/s]")
    ax_ang.set_title("Base angular velocity")
    ax_ang.grid(True, alpha=0.3)
    ax_ang.legend(ncol=3, fontsize=8)

    com = traj["com_pos"]
    ee = traj["ee_pos"]
    ax_3d.plot(com[:, 0], com[:, 1], com[:, 2], color="tab:green", label="COM")
    ax_3d.plot(ee[:, 0], ee[:, 1], ee[:, 2], color="tab:blue", label="EE")
    ax_3d.scatter(
        ee_goal[0],
        ee_goal[1],
        ee_goal[2],
        color="gold",
        marker="*",
        s=120,
        label="EE goal",
    )
    ax_3d.set_xlabel("x [m]")
    ax_3d.set_ylabel("y [m]")
    ax_3d.set_zlabel("z [m]")
    ax_3d.set_title("COM and end-effector position")
    ax_3d.legend(fontsize=8)

    ax_err.plot(times, traj["ee_errors"], label=f"||p_ee - p_goal|| ({CHEWIE_EE_FRAME})")
    ax_err.set_xlabel("time [s]")
    ax_err.set_ylabel("error [m]")
    ax_err.set_title("End-effector tracking error")
    ax_err.grid(True, alpha=0.3)
    ax_err.legend(fontsize=8)

    fig.suptitle("Figure B: base and end-effector tracking")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "figure_b.png"), dpi=150, bbox_inches="tight")
    _write_csv(os.path.join(out_dir, "figure_b.csv"), csv_columns, csv_rows)
    plt.close(fig)


def save_rss_reach_outputs(xs, model, data, dt, ee_frame_id, ee_goal, q_ref, urdf_path, out_dir):
    traj = _extract_trajectory_data(xs, model, data, dt, ee_frame_id, ee_goal)
    limits = parse_urdf_joint_limits(urdf_path, list(ARM_INIT_POS.keys()))
    _save_arm_joint_figure(out_dir, traj, limits, "han", HAN_JOINTS, "tab:blue", "-")
    _save_arm_joint_figure(
        out_dir, traj, limits, "chewie", CHEWIE_JOINTS, "tab:red", "-"
    )
    _save_figure_b(out_dir, traj, q_ref, ee_goal)
    print(f"Saved figures and CSVs to {os.path.abspath(out_dir)}")


# ====================================================================
# Defining the reach goal (position only)
# ====================================================================

nu = state.nv  # cost/control dimension expected by crocoddyl cost sums

# Chewie end-effector frame and desired position in the env / world frame.
CHEWIE_EE_FRAME = "chewie_tool0"
CHEWIE_EE_ID = model.getFrameId(CHEWIE_EE_FRAME)
CHEWIE_EE_POS_ENV = np.array([2.0, 0.0, 0.0])

eeResidual = crocoddyl.ResidualModelFrameTranslation(
    state, CHEWIE_EE_ID, CHEWIE_EE_POS_ENV, nu
)
eeActivation = crocoddyl.ActivationModelQuad(3)
eeCost = crocoddyl.CostModelResidual(state, eeActivation, eeResidual)
eeRunCost = crocoddyl.CostModelResidual(state, eeActivation, eeResidual)

# ====================================================================
# Setting up the running state and control cost models
# ====================================================================

# Actuation layout (16): chewie arm+gripper [0:8], han arm+gripper [8:16].
# Both arms may actuate; Han is steered by the running base-stabilization cost.
w_u = np.ones(actuation.nu) * 0.01
uResidual = crocoddyl.ResidualModelJointEffort(
    state, actuation, np.zeros(actuation.nu), nu, False
)
uRegCost = crocoddyl.CostModelResidual(
    state, crocoddyl.ActivationModelWeightedQuad(w_u), uResidual
)

# Running-only: damp base translation and joint velocities; Han compensates via
# base position tracking. Base orientation / angular rate are unconstrained.
xRef = state.zero()
xRef[: model.nq] = q0
w_base = np.zeros(state.ndx)
w_base[0:3] = 1e2
w_base[state.nv : state.nv + 3] = 10
w_base[state.nv + 6 :] = 10
baseRunResidual = crocoddyl.ResidualModelState(state, xRef, nu)
baseRunActivation = crocoddyl.ActivationModelWeightedQuad(w_base)
baseRunCost = crocoddyl.CostModelResidual(state, baseRunActivation, baseRunResidual)

# Chewie's shoulder_lift can reach the goal by folding "down" (decreasing angle)
# straight through the body/other arm, or by swinging "up and over" (increasing
# angle). Push the optimizer into the up-and-over homotopy with a one-sided
# barrier that penalizes the joint dropping below its start angle.
lift_jid = model.getJointId("chewie_shoulder_lift_joint")
lift_idx_q = model.joints[lift_jid].idx_q
lift_idx_v = model.joints[lift_jid].idx_v

xLiftRef = state.zero()
xLiftRef[: model.nq] = q0  # lift component already sits at its start angle
liftLower = np.full(state.ndx, -np.inf)
liftUpper = np.full(state.ndx, np.inf)
liftLower[lift_idx_v] = 0.0  # residual = q_lift - q_lift_start >= 0
liftBarrierResidual = crocoddyl.ResidualModelState(state, xLiftRef, nu)
liftBarrierActivation = crocoddyl.ActivationModelQuadraticBarrier(
    crocoddyl.ActivationBounds(liftLower, liftUpper)
)
liftBarrierCost = crocoddyl.CostModelResidual(
    state, liftBarrierActivation, liftBarrierResidual
)

# ====================================================================
# Setting up the running and terminal cost models
# ====================================================================

CONTROL_HZ = 50
dt = 1.0 / CONTROL_HZ

runningCostModel = crocoddyl.CostModelSum(state, nu)
runningCostModel.addCost("uReg", uRegCost, 1 / dt)
runningCostModel.addCost("baseRun", baseRunCost, 10.0)
runningCostModel.addCost("eeRun", eeRunCost, 10.0)
runningCostModel.addCost("chewieLiftBarrier", liftBarrierCost, 1e3)

terminalCostModel = crocoddyl.CostModelSum(state, nu)
terminalCostModel.addCost("ee", eeCost, 1e6)

runningModel = crocoddyl.IntegratedActionModelEuler(
    crocoddyl.DifferentialActionModelFreeInvDynamics(
        state, actuation, runningCostModel
    ),
    dt,
)
terminalModel = crocoddyl.IntegratedActionModelEuler(
    crocoddyl.DifferentialActionModelFreeInvDynamics(
        state, actuation, terminalCostModel
    ),
    dt,
)

# ====================================================================
# Creating the shooting problem and the solver
# ====================================================================

HORIZON_S = 5.0
T = int(round(HORIZON_S / dt))
x0 = state.zero()
x0[: model.nq] = q0
problem = crocoddyl.ShootingProblem(x0, [runningModel] * T, terminalModel)
solver = crocoddyl.SolverIntro(problem)

solver.setCallbacks([crocoddyl.CallbackVerbose()])

# Warm-start in the "up-and-over" homotopy: seed a trajectory in which Chewie's
# shoulder_lift ramps upward (increasing angle) rather than folding down through
# the body. Without this, gradient descent from the default constant guess lands
# in the fold-down local minimum and merely pays the barrier penalty.
xs_init = [x0.copy() for _ in range(T + 1)]
for k, x in enumerate(xs_init):
    x[lift_idx_q] = q0[lift_idx_q] + 2.0 * (k / T)
us_init = solver.problem.quasiStatic([x for x in xs_init[:-1]])

# Solving the problem with the solver
solver.solve(xs_init, us_init)

q_final = solver.xs[-1][: model.nq]
pinocchio.forwardKinematics(model, data, q_final)
pinocchio.updateFramePlacement(model, data, CHEWIE_EE_ID)
ee_final = data.oMf[CHEWIE_EE_ID].translation.copy()
print(f"Target {CHEWIE_EE_FRAME} position: {CHEWIE_EE_POS_ENV}")
print(f"Final   {CHEWIE_EE_FRAME} position: {ee_final}")
print(f"Position error [m]: {np.linalg.norm(ee_final - CHEWIE_EE_POS_ENV):.4f}")
print(f"Base position drift [m]: {np.linalg.norm(q_final[:3] - q0[:3]):.4f}")
print(f"Base orientation drift [quat]: {q_final[3:7] - q0[3:7]}")

_lift_traj = np.array([x[lift_idx_q] for x in solver.xs])
print(
    f"chewie shoulder_lift: start={_lift_traj[0]:.3f} "
    f"min={_lift_traj.min():.3f} max={_lift_traj.max():.3f} end={_lift_traj[-1]:.3f}"
)

# ====================================================================
# Displaying the motion
# ====================================================================

# Plotting the entire motion
if WITHPLOT:
    out_dir = _make_output_dir()
    save_rss_reach_outputs(
        solver.xs,
        model,
        data,
        dt,
        CHEWIE_EE_ID,
        CHEWIE_EE_POS_ENV,
        q0,
        urdf_path,
        out_dir,
    )

# Displaying the motion in Meshcat
if WITHDISPLAY or WITHVIDEO:
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=WITHDISPLAY)
    viz.loadViewerModel()
    viz.display(q0)

    qs = [x[: model.nq] for x in solver.xs]
    pause = 2.0
    frame_dir = None
    frame_idx = 0
    if WITHVIDEO:
        frame_dir = os.path.join(
            os.path.dirname(__file__), "..", "output", "rss_reach_frames"
        )
        os.makedirs(frame_dir, exist_ok=True)
        print(f"Saving frames to {os.path.abspath(frame_dir)}")
        print(
            "Create video with:\n"
            f"  ffmpeg -y -framerate {1 / dt:.0f} "
            f"-i {os.path.abspath(frame_dir)}/frame_%06d.png "
            "-c:v libx264 -pix_fmt yuv420p rss_reach.mp4"
        )

    save_frames = WITHVIDEO
    while True:
        for q in qs:
            viz.display(q)
            if save_frames:
                plt.imsave(
                    os.path.join(frame_dir, f"frame_{frame_idx:06d}.png"),
                    viz.captureImage(),
                )
                frame_idx += 1
            time.sleep(dt)
        save_frames = False
        time.sleep(pause)
