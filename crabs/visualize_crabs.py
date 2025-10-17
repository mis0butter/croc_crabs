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

# Create data
data = model.createData()

# Create the Crocoddyl state
state = crocoddyl.StateMultibody(model)

# Create the Crocoddyl actuation model 
actuation = crocoddyl.ActuationModelFloatingBase(state) 

# ==================================================================== 
# Display the robot 
# ==================================================================== 
 
viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)   # launches MeshCat in your browser
viz.loadViewerModel()
viz.display(np.zeros(model.nq)) 

while True: 
    q = np.zeros(model.nq)
    viz.display(q)
    time.sleep(0.01)

