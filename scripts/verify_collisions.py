import os 

import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
from pinocchio import JointModelFreeFlyer, GeometryType

# ==================================================================== 
# ==================================================================== 

# Define paths
model_dir = os.path.join(os.path.dirname(__file__), "..", "hcrl_crab_robot")
urdf_path = os.path.join(model_dir, "crab.urdf")

# Optional: add package directory for meshes
package_dirs = [model_dir]

model = pin.buildModelFromUrdf(urdf_path, JointModelFreeFlyer())
collision_model = pin.buildGeomFromUrdf(model, urdf_path, GeometryType.COLLISION, package_dirs)
visual_model    = pin.buildGeomFromUrdf(model, urdf_path, GeometryType.VISUAL,    package_dirs)

# 1) Detect/repair duplicate geometry object names (Meshcat path keys)
#    Prefix each collision object with its parent joint name to ensure
#    uniqueness across the whole tree; then deduplicate if a name still clashes.
seen = set()
for go in collision_model.geometryObjects:
    base = f"{model.names[go.parentJoint]}__{go.name}"
    name = base
    k = 1
    while name in seen:
        name = f"{base}_{k}"
        k += 1
    go.name = name
    seen.add(name)

# 1b) Promote collisions to visuals so Meshcat renders them as standard visuals
#     (helps when collisions are hidden/overwritten in the collisions group).
for go in collision_model.geometryObjects:
    vcopy = pin.GeometryObject(go)
    vcopy.name = f"colvis__{go.name}"
    visual_model.addGeometryObject(vcopy)

# 2) Clean and (re)load
viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)
viz.clean()
viz.loadViewerModel()
viz.display(pin.neutral(model))
viz.displayCollisions(False)
viz.displayVisuals(True)

print("collision count:", len(collision_model.geometryObjects))
for i, go in enumerate(collision_model.geometryObjects):
    parent = model.names[go.parentJoint]
    print(f"{i:02d} name={go.name} parent={parent} placement={go.placement}")