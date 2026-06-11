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
# Root free flyer is unactuated; UR20 arm (+ gripper) joints are actuated.
actuation = crocoddyl.ActuationModelFloatingBase(state)

# ====================================================================
# Defining the goal state and cost
# ====================================================================

nu = state.nv  # cost/control dimension expected by crocoddyl cost sums

# Target: reorient the floating base by 20 deg about Y using arm torques only.
theta = np.deg2rad(20.0)
q_goal = pinocchio.neutral(model)

# q layout for free-flyer: [x y z  qx qy qz qw]
q_goal[3:7] = np.array([0.0, np.sin(theta / 2.0), 0.0, np.cos(theta / 2.0)])

# Define the goal state
xGoal = state.zero()
xGoal[: model.nq] = q_goal

# Weight the floating-base orientation error most strongly.
w = np.ones(state.ndx)
w[3:6] = 1e4
w[state.nv + 3 : state.nv + 6] = 100

xGoalActivation = crocoddyl.ActivationModelWeightedQuad(w)
xGoalResidual = crocoddyl.ResidualModelState(state, xGoal, nu)
xGoalCost = crocoddyl.CostModelResidual(state, xGoalActivation, xGoalResidual)

# ====================================================================
# Setting up the running state and control cost models
# ====================================================================

w_u = np.ones(actuation.nu) * 0.1
uResidual = crocoddyl.ResidualModelJointEffort(
    state, actuation, np.zeros(actuation.nu), nu, False
)
uRegCost = crocoddyl.CostModelResidual(
    state, crocoddyl.ActivationModelWeightedQuad(w_u), uResidual
)

xRunCost = crocoddyl.CostModelResidual(state, xGoalActivation, xGoalResidual)

# ====================================================================
# Setting up the running and terminal cost models
# ====================================================================

dt = 1e-2

runningCostModel = crocoddyl.CostModelSum(state, nu)
runningCostModel.addCost("uReg", uRegCost, 10 / dt)
runningCostModel.addCost("xGoalTerm", xGoalCost, 1e-3 / dt)
runningCostModel.addCost("xRun", xRunCost, 10.0)

terminalCostModel = crocoddyl.CostModelSum(state, nu)
terminalCostModel.addCost("xGoalTerm", xGoalCost, 1e5)

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
    viz.display(pinocchio.neutral(model))

    qs = [x[: model.nq] for x in solver.xs]
    pause = 2.0
    while True:
        for q in qs:
            viz.display(q)
            time.sleep(dt)
        time.sleep(pause)
