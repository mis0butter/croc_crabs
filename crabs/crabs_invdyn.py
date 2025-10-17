# ref. https://github.com/PyCQA/pycodestyle/issues/373, remove this for ruff
import os
import signal
import sys
import time

import numpy as np
import pinocchio
from pinocchio.visualize import MeshcatVisualizer 
from pinocchio import JointModelFreeFlyer

import crocoddyl

# ==================================================================== 
# Loading the crab robot model
# ==================================================================== 

# argument handling 
WITHDISPLAY = "display" in sys.argv or "CROCODDYL_DISPLAY" in os.environ
WITHPLOT = "plot" in sys.argv or "CROCODDYL_PLOT" in os.environ
signal.signal(signal.SIGINT, signal.SIG_DFL)

# Define paths
model_dir = os.path.join(os.path.dirname(__file__), "hcrl_crab_robot")
urdf_path = os.path.join(model_dir, "crab.urdf")

# Optional: add package directory for meshes
package_dirs = [model_dir]

# Load model + visuals
model, collision_model, visual_model = pinocchio.buildModelsFromUrdf(urdf_path, package_dirs, JointModelFreeFlyer())

# turn off gravity!  
model.gravity.linear[:] = 0.0 

# Create data
data = model.createData()

# Create the Crocoddyl state
state = crocoddyl.StateMultibody(model)

# Create the Crocoddyl actuation model 
actuation = crocoddyl.ActuationModelFloatingBase(state)

# ==================================================================== 
# Defining the cost model
# ==================================================================== 

nu = state.nv # number of controls
# nu = actuation.nu 
runningCostModel = crocoddyl.CostModelSum(state, nu)  
terminalCostModel = crocoddyl.CostModelSum(state, nu) 

# Define the residual model for the state
xResidual = crocoddyl.ResidualModelState(state, state.zero(), nu)

# Define the activation model for the state 
xActivation = crocoddyl.ActivationModelQuad(state.ndx)

# Define the residual model for the controls
uResidual = crocoddyl.ResidualModelJointEffort(
    state, actuation, np.zeros(actuation.nu), nu, False
)

# Define the cost model for the state
xRegCost = crocoddyl.CostModelResidual(state, xActivation, xResidual)
uRegCost = crocoddyl.CostModelResidual(state, uResidual)

# Define a goal state 
# xGoal = np.zeros(state.nx)
xGoal = state.zero() 

# Define the activation model for the goal state
xGoalActivation = crocoddyl.ActivationModelWeightedQuad(np.ones(state.ndx))

# Define the residual model for the goal state
xGoalResidual = crocoddyl.ResidualModelState(state, xGoal, nu)

# Define the cost model for the goal state
xPendCost = crocoddyl.CostModelResidual(
    state, 
    xGoalActivation, 
    xGoalResidual
)

dt = 1e-2

runningCostModel.addCost("uReg", uRegCost, 1e-4 / dt)
runningCostModel.addCost("xGoal", xPendCost, 1e-5 / dt)
terminalCostModel.addCost("xGoal", xPendCost, 100.0)

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

T = 100
# Initialize with a reasonable starting configuration for the crab robot
# x0 = np.zeros(state.nx)
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
    viz.initViewer(open=True)   # launches MeshCat in your browser
    viz.loadViewerModel()
    viz.display(np.zeros(model.nq))

    # After solving:
    for x in solver.xs:
        q = x[:model.nq]
        viz.display(q)
        time.sleep(0.01)

