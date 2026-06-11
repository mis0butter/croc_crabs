# ref. https://github.com/PyCQA/pycodestyle/issues/373, remove this for ruff
import os
import signal
import sys
import tempfile
import time

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

# ====================================================================
# Setting up the running and terminal cost models
# ====================================================================

dt = 1e-2

runningCostModel = crocoddyl.CostModelSum(state, nu)
runningCostModel.addCost("uReg", uRegCost, 1 / dt)
runningCostModel.addCost("baseRun", baseRunCost, 10.0)
runningCostModel.addCost("eeRun", eeRunCost, 10.0)

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

T = 500
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

# Solving the problem with the solver
solver.solve()

q_final = solver.xs[-1][: model.nq]
pinocchio.forwardKinematics(model, data, q_final)
pinocchio.updateFramePlacement(model, data, CHEWIE_EE_ID)
ee_final = data.oMf[CHEWIE_EE_ID].translation.copy()
print(f"Target {CHEWIE_EE_FRAME} position: {CHEWIE_EE_POS_ENV}")
print(f"Final   {CHEWIE_EE_FRAME} position: {ee_final}")
print(f"Position error [m]: {np.linalg.norm(ee_final - CHEWIE_EE_POS_ENV):.4f}")
print(f"Base position drift [m]: {np.linalg.norm(q_final[:3] - q0[:3]):.4f}")
print(f"Base orientation drift [quat]: {q_final[3:7] - q0[3:7]}")

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

# Displaying the motion in Meshcat
if WITHDISPLAY:
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True)
    viz.loadViewerModel()
    viz.display(q0)

    qs = [x[: model.nq] for x in solver.xs]
    pause = 2.0
    while True:
        for q in qs:
            viz.display(q)
            time.sleep(dt)
        time.sleep(pause)
