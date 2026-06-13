# ref. https://github.com/PyCQA/pycodestyle/issues/373, remove this for ruff
import os
import signal
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

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


def _joint_label(joint_name):
    return joint_name.replace("han_", "").replace("chewie_", "").replace("_joint", "")


def _plot_arm_joint_positions(times, positions, joint_names, limits, title, fig_num):
    n = len(joint_names)
    ncols = 4
    nrows = (n + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, num=fig_num, figsize=(14, 2.5 * nrows), sharex=True)
    axes = np.atleast_1d(axes).ravel()
    for ax, joint_name in zip(axes, joint_names):
        ax.plot(times, positions[joint_name], label="position")
        if joint_name in limits:
            lo, hi = limits[joint_name]["lower"], limits[joint_name]["upper"]
            ax.axhline(lo, color="C3", ls="--", lw=1, label="limit")
            ax.axhline(hi, color="C3", ls="--", lw=1)
        ax.set_title(_joint_label(joint_name))
        ax.grid(True, alpha=0.3)
    for ax in axes[n:]:
        ax.set_visible(False)
    axes[0].legend(loc="upper right", fontsize=8)
    fig.suptitle(title)
    fig.tight_layout()


def _plot_arm_joint_velocities(times, velocities, joint_names, limits, title, fig_num):
    n = len(joint_names)
    ncols = 4
    nrows = (n + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, num=fig_num, figsize=(14, 2.5 * nrows), sharex=True)
    axes = np.atleast_1d(axes).ravel()
    for ax, joint_name in zip(axes, joint_names):
        ax.plot(times, velocities[joint_name], label="velocity")
        if joint_name in limits:
            vlim = limits[joint_name]["velocity"]
            ax.axhline(vlim, color="C3", ls="--", lw=1, label="limit")
            ax.axhline(-vlim, color="C3", ls="--", lw=1)
        ax.set_title(_joint_label(joint_name))
        ax.grid(True, alpha=0.3)
    for ax in axes[n:]:
        ax.set_visible(False)
    axes[0].legend(loc="upper right", fontsize=8)
    fig.suptitle(title)
    fig.tight_layout()


def plot_rss_reach_trajectory(
    xs, model, data, dt, ee_frame_id, ee_goal, urdf_path, q_ref, fig_start=3
):
    joint_limits = parse_urdf_joint_limits(urdf_path, list(ARM_INIT_POS.keys()))
    times = np.arange(len(xs)) * dt

    positions = {name: [] for name in ARM_INIT_POS}
    velocities = {name: [] for name in ARM_INIT_POS}
    ee_errors = []
    base_omega = []
    base_rpy = []

    for x in xs:
        q = x[: model.nq]
        v = x[model.nq :]
        for joint_name in ARM_INIT_POS:
            jid = model.getJointId(joint_name)
            idx_q = model.joints[jid].idx_q
            idx_v = model.joints[jid].idx_v
            positions[joint_name].append(q[idx_q])
            velocities[joint_name].append(v[idx_v])

        pinocchio.forwardKinematics(model, data, q)
        pinocchio.updateFramePlacement(model, data, ee_frame_id)
        ee_pos = data.oMf[ee_frame_id].translation
        ee_errors.append(np.linalg.norm(ee_pos - ee_goal))

        quat = pinocchio.Quaternion(q[6], q[3], q[4], q[5])
        base_rpy.append(pinocchio.rpy.matrixToRpy(quat.toRotationMatrix()))
        base_omega.append(v[3:6])

    positions = {name: np.asarray(vals) for name, vals in positions.items()}
    velocities = {name: np.asarray(vals) for name, vals in velocities.items()}
    base_rpy = np.asarray(base_rpy)
    base_omega = np.asarray(base_omega)
    rpy_ref = pinocchio.rpy.matrixToRpy(
        pinocchio.Quaternion(q_ref[6], q_ref[3], q_ref[4], q_ref[5]).toRotationMatrix()
    )

    _plot_arm_joint_positions(
        times,
        positions,
        HAN_JOINTS,
        joint_limits,
        "Han joint positions",
        fig_start,
    )
    _plot_arm_joint_positions(
        times,
        positions,
        CHEWIE_JOINTS,
        joint_limits,
        "Chewie joint positions",
        fig_start + 1,
    )
    _plot_arm_joint_velocities(
        times,
        velocities,
        HAN_JOINTS,
        joint_limits,
        "Han joint velocities",
        fig_start + 2,
    )
    _plot_arm_joint_velocities(
        times,
        velocities,
        CHEWIE_JOINTS,
        joint_limits,
        "Chewie joint velocities",
        fig_start + 3,
    )

    fig, ax = plt.subplots(num=fig_start + 4, figsize=(8, 3))
    ax.plot(times, ee_errors, label="||p_ee - p_goal||")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("position error [m]")
    ax.set_title(f"End-effector tracking error ({CHEWIE_EE_FRAME})")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    fig, axes = plt.subplots(2, 1, num=fig_start + 5, figsize=(8, 5), sharex=True)
    rpy_labels = ["roll", "pitch", "yaw"]
    for i, label in enumerate(rpy_labels):
        axes[0].plot(times, base_rpy[:, i], label=label)
        axes[0].axhline(rpy_ref[i], color=f"C{i}", ls=":", lw=1, alpha=0.7)
    axes[0].set_ylabel("orientation [rad]")
    axes[0].set_title("Base orientation (RPY)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(ncol=3, fontsize=8)

    omega_labels = ["wx", "wy", "wz"]
    for i, label in enumerate(omega_labels):
        axes[1].plot(times, base_omega[:, i], label=label)
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("angular velocity [rad/s]")
    axes[1].set_title("Base angular velocity")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(ncol=3, fontsize=8)
    fig.tight_layout()


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

CONTROL_HZ = 250
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

if WITHPLOT:
    solver.setCallbacks(
        [
            crocoddyl.CallbackVerbose(),
            crocoddyl.CallbackLogger(),
        ]
    )
else:
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
    log = solver.getCallbacks()[1]
    crocoddyl.plotOCSolution(
        log.xs, [u[state.nv :] for u in log.us], figIndex=1, show=False
    )
    crocoddyl.plotConvergence(
        log.costs, log.pregs, log.dregs, log.grads, log.stops, log.steps, figIndex=2
    )

plot_rss_reach_trajectory(
    solver.xs,
    model,
    data,
    dt,
    CHEWIE_EE_ID,
    CHEWIE_EE_POS_ENV,
    urdf_path,
    q0,
    fig_start=10 if WITHPLOT else 1,
)
plt.show(block=not (WITHDISPLAY or WITHVIDEO))

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
