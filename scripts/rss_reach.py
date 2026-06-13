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


def parse_armed_mockup_inertial(urdf_path):
    for link in ET.parse(urdf_path).getroot().findall("link"):
        if link.get("name") != "armed_mockup":
            continue
        inertial = link.find("inertial")
        if inertial is None:
            break
        mass = inertial.find("mass")
        inertia = inertial.find("inertia")
        return {
            "link": "armed_mockup",
            "mass_kg": float(mass.get("value")),
            "ixx": float(inertia.get("ixx")),
            "ixy": float(inertia.get("ixy")),
            "ixz": float(inertia.get("ixz")),
            "iyy": float(inertia.get("iyy")),
            "iyz": float(inertia.get("iyz")),
            "izz": float(inertia.get("izz")),
        }
    return None


def _csv_cell(value):
    text = np.array2string(value, separator=" ") if isinstance(value, np.ndarray) else str(value)
    if any(c in text for c in [",", '"', "\n"]):
        return '"' + text.replace('"', '""') + '"'
    return text


def _save_sim_parameters(out_dir, urdf_path, sim_params):
    inertial = parse_armed_mockup_inertial(urdf_path)
    md_path = os.path.join(out_dir, "sim_parameters.md")
    csv_path = os.path.join(out_dir, "sim_parameters.csv")

    lines = [
        "# RSS reach simulation parameters",
        "",
        "## Problem setup",
        "",
        f"- control_hz: {sim_params['control_hz']}",
        f"- dt [s]: {sim_params['dt']}",
        f"- horizon_s [s]: {sim_params['horizon_s']}",
        f"- T (nodes): {sim_params['T']}",
        f"- gravity [m/s^2]: {np.array2string(sim_params['gravity'], separator=' ')}",
        f"- ee_frame: {sim_params['ee_frame']}",
        f"- ee_goal [m]: {np.array2string(sim_params['ee_goal'], separator=' ')}",
        "",
        "## Armed mockup inertial (`armed_mockup` link, URDF)",
        "",
    ]
    csv_rows = [
        ("section", "parameter", "value"),
        ("problem", "control_hz", sim_params["control_hz"]),
        ("problem", "dt_s", sim_params["dt"]),
        ("problem", "horizon_s", sim_params["horizon_s"]),
        ("problem", "T", sim_params["T"]),
        ("problem", "gravity_m_s2", sim_params["gravity"]),
        ("problem", "ee_frame", sim_params["ee_frame"]),
        ("problem", "ee_goal_m", sim_params["ee_goal"]),
    ]

    if inertial:
        lines.extend(
            [
                f"- mass [kg]: {inertial['mass_kg']}",
                f"- ixx [kg m^2]: {inertial['ixx']}",
                f"- ixy [kg m^2]: {inertial['ixy']}",
                f"- ixz [kg m^2]: {inertial['ixz']}",
                f"- iyy [kg m^2]: {inertial['iyy']}",
                f"- iyz [kg m^2]: {inertial['iyz']}",
                f"- izz [kg m^2]: {inertial['izz']}",
                "",
            ]
        )
        for key in ("mass_kg", "ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            csv_rows.append(("armed_mockup_inertial", key, inertial[key]))
    else:
        lines.append("- armed_mockup inertial not found in URDF")
        lines.append("")

    lines.extend(
        [
            "## Running cost terms",
            "",
            "| term | weight | description |",
            "| --- | ---: | --- |",
        ]
    )
    for cost in sim_params["running_costs"]:
        lines.append(f"| {cost['name']} | {cost['weight']} | {cost['description']} |")
        csv_rows.append(("running_cost", cost["name"], cost["weight"]))
        csv_rows.append(("running_cost_description", cost["name"], cost["description"]))

    lines.extend(
        [
            "",
            "## Terminal cost terms",
            "",
            "| term | weight | description |",
            "| --- | ---: | --- |",
        ]
    )
    for cost in sim_params["terminal_costs"]:
        lines.append(f"| {cost['name']} | {cost['weight']} | {cost['description']} |")
        csv_rows.append(("terminal_cost", cost["name"], cost["weight"]))
        csv_rows.append(("terminal_cost_description", cost["name"], cost["description"]))

    lines.extend(
        [
            "",
            "## Internal activation weights",
            "",
            "### Control regularization (`uReg`)",
            "",
            f"- w_u (all {len(sim_params['w_u'])} actuated joints): {sim_params['w_u'][0]}",
            "",
            "### Base stabilization (`baseRun`, running only)",
            "",
            f"- w_base[0:3] base position: {sim_params['w_base_position']}",
            f"- w_base[nv:nv+3] base linear velocity: {sim_params['w_base_lin_vel']}",
            f"- w_base[nv+6:] arm joint velocities: {sim_params['w_base_joint_vel']}",
            "",
            "### Chewie lift barrier (`chewieLiftBarrier`)",
            "",
            f"- lower bound on chewie_shoulder_lift velocity index: {sim_params['lift_barrier_lower']}",
            "  (penalizes shoulder_lift dropping below its start angle)",
            "",
        ]
    )
    csv_rows.extend(
        [
            ("activation", "w_u", sim_params["w_u"][0]),
            ("activation", "w_base_position", sim_params["w_base_position"]),
            ("activation", "w_base_lin_vel", sim_params["w_base_lin_vel"]),
            ("activation", "w_base_joint_vel", sim_params["w_base_joint_vel"]),
            ("activation", "lift_barrier_lower", sim_params["lift_barrier_lower"]),
        ]
    )

    with open(md_path, "w") as f:
        f.write("\n".join(lines) + "\n")

    with open(csv_path, "w") as f:
        f.write("section,parameter,value\n")
        for section, parameter, value in csv_rows[1:]:
            f.write(f"{section},{parameter},{_csv_cell(value)}\n")

    print(f"Saved sim parameters to {os.path.abspath(md_path)}")


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


def _save_base_figure(out_dir, traj, q_ref):
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
    ]
    csv_rows = [
        [t, *traj["base_rpy"][k], *traj["base_omega"][k]]
        for k, t in enumerate(times)
    ]

    fig, (ax_ori, ax_ang) = plt.subplots(1, 2, figsize=(12, 4))

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

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "base_motion.png"), dpi=150, bbox_inches="tight")
    _write_csv(os.path.join(out_dir, "base_motion.csv"), csv_columns, csv_rows)
    plt.close(fig)


def _save_com_ee_figure(out_dir, traj, ee_goal):
    times = traj["times"]
    com = traj["com_pos"]
    ee = traj["ee_pos"]

    csv_columns = [
        "time_s",
        "com_x",
        "com_y",
        "com_z",
        "ee_x",
        "ee_y",
        "ee_z",
        "ee_error",
    ]
    csv_rows = [
        [t, *com[k], *ee[k], traj["ee_errors"][k]]
        for k, t in enumerate(times)
    ]

    fig = plt.figure(figsize=(12, 9))
    gs = fig.add_gridspec(3, 2, width_ratios=[1, 1.2], hspace=0.35, wspace=0.3)

    ax_com_x = fig.add_subplot(gs[0, 0])
    ax_com_y = fig.add_subplot(gs[1, 0], sharex=ax_com_x)
    ax_com_z = fig.add_subplot(gs[2, 0], sharex=ax_com_x)
    ax_3d = fig.add_subplot(gs[0:2, 1], projection="3d")
    ax_err = fig.add_subplot(gs[2, 1], sharex=ax_com_x)

    ax_com_x.plot(times, com[:, 0], color="tab:green")
    ax_com_x.set_ylabel("x [m]")
    ax_com_x.set_title("COM x")
    ax_com_x.grid(True, alpha=0.3)

    ax_com_y.plot(times, com[:, 1], color="tab:green")
    ax_com_y.set_ylabel("y [m]")
    ax_com_y.set_title("COM y")
    ax_com_y.grid(True, alpha=0.3)

    ax_com_z.plot(times, com[:, 2], color="tab:green")
    ax_com_z.set_xlabel("time [s]")
    ax_com_z.set_ylabel("z [m]")
    ax_com_z.set_title("COM z")
    ax_com_z.grid(True, alpha=0.3)

    plt.setp(ax_com_x.get_xticklabels(), visible=False)
    plt.setp(ax_com_y.get_xticklabels(), visible=False)

    ax_err.plot(
        times,
        traj["ee_errors"],
        color="tab:purple",
        label=f"||p_ee - p_goal|| ({CHEWIE_EE_FRAME})",
    )
    ax_err.set_xlabel("time [s]")
    ax_err.set_ylabel("error [m]")
    ax_err.set_title("End-effector tracking error")
    ax_err.grid(True, alpha=0.3)
    ax_err.legend(fontsize=8)

    ax_3d.scatter(
        com[:, 0],
        com[:, 1],
        com[:, 2],
        color="tab:green",
        marker="o",
        s=40,
        label="COM",
    )
    ax_3d.plot(ee[:, 0], ee[:, 1], ee[:, 2], color="tab:purple", label="EE")
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

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "com_ee_tracking.png"), dpi=150, bbox_inches="tight")
    _write_csv(os.path.join(out_dir, "com_ee_tracking.csv"), csv_columns, csv_rows)
    plt.close(fig)


def save_rss_reach_outputs(xs, model, data, dt, ee_frame_id, ee_goal, q_ref, urdf_path, out_dir, sim_params):
    traj = _extract_trajectory_data(xs, model, data, dt, ee_frame_id, ee_goal)
    limits = parse_urdf_joint_limits(urdf_path, list(ARM_INIT_POS.keys()))
    _save_arm_joint_figure(out_dir, traj, limits, "han", HAN_JOINTS, "tab:blue", "-")
    _save_arm_joint_figure(
        out_dir, traj, limits, "chewie", CHEWIE_JOINTS, "tab:red", "-"
    )
    _save_base_figure(out_dir, traj, q_ref)
    _save_com_ee_figure(out_dir, traj, ee_goal)
    _save_sim_parameters(out_dir, urdf_path, sim_params)
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
    sim_params = {
        "control_hz": CONTROL_HZ,
        "dt": dt,
        "horizon_s": HORIZON_S,
        "T": T,
        "gravity": model.gravity.linear.copy(),
        "ee_frame": CHEWIE_EE_FRAME,
        "ee_goal": CHEWIE_EE_POS_ENV,
        "running_costs": [
            {
                "name": "uReg",
                "weight": 1 / dt,
                "description": "Actuation regularization (WeightedQuad, w_u=0.01 per joint)",
            },
            {
                "name": "baseRun",
                "weight": 10.0,
                "description": "Track initial base pose and damp base/arm joint velocities",
            },
            {
                "name": "eeRun",
                "weight": 10.0,
                "description": f"Running EE position tracking ({CHEWIE_EE_FRAME})",
            },
            {
                "name": "chewieLiftBarrier",
                "weight": 1e3,
                "description": "One-sided barrier: chewie shoulder_lift >= start angle",
            },
        ],
        "terminal_costs": [
            {
                "name": "ee",
                "weight": 1e6,
                "description": f"Terminal EE position tracking ({CHEWIE_EE_FRAME})",
            },
        ],
        "w_u": w_u,
        "w_base_position": w_base[0],
        "w_base_lin_vel": w_base[state.nv],
        "w_base_joint_vel": w_base[state.nv + 6],
        "lift_barrier_lower": liftLower[lift_idx_v],
    }
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
        sim_params,
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
