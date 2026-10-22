# RSS reach simulation parameters

## Problem setup

- control_hz: 50
- dt [s]: 0.02
- horizon_s [s]: 5.0
- T (nodes): 250
- gravity [m/s^2]: [0. 0. 0.]
- ee_frame: chewie_tool0
- ee_goal [m]: [2. 0. 0.]

## Armed mockup inertial (`armed_mockup` link, URDF)

- mass [kg]: 3000.0
- ixx [kg m^2]: 499.9998
- ixy [kg m^2]: 0.0
- ixz [kg m^2]: 0.0
- iyy [kg m^2]: 499.9998
- iyz [kg m^2]: 0.0
- izz [kg m^2]: 499.9998

## Force/torque sensor wrench

Each sensor frame sits between the optical plate and the arm tombstone, so it
carries the whole arm above it. Gravity is zero in the model, so the reported
wrench is purely motion-induced and is directly comparable to a
gravity-compensated hardware measurement. The final sample repeats the last
commanded acceleration, since there is one more state than control.

| frame | hardware | arm mass [kg] | origin in base frame [m] |
| --- | --- | ---: | --- |
| `han_fts` | ATI Omega191 | 72.9217 | [0.2667 0.2921 0.0894] |
| `chewie_fts_combined` | Bota LAX triplet (combined) | 81.0698 | [ 0.2286 -0.2794  0.0699] |

### Column conventions in `<frame>_wrench.csv`

- Torques are taken about the origin of the named frame.
- `local`: components in the sensor frame's own axes.
- `world_aligned`: same origin, components rotated into the inertial frame.
- `mount_on_arm`: force the mount/table side applies to the arm assembly
  (Pinocchio/RNEA convention).
- `arm_on_mount`: the reaction, i.e. what the arm pushes into the sensor.
  Equal to `-mount_on_arm`.

### Validation

- max |unactuated floating-base residual| over the commanded nodes: 1.474e-11 (N and N m; should be ~0). The terminal node in `validation.csv` is flagged `commanded=0` because it extrapolates the last command and is not constrained by the solver.
- max wrench disagreement against an independent model with real joints at the sensors: 3.553e-15 against a peak magnitude of 18.474

## Running cost terms

| term | weight | description |
| --- | ---: | --- |
| uReg | 50.0 | Actuation regularization (WeightedQuad, w_u=0.01 per joint) |
| baseRun | 10.0 | Track initial base pose and damp base/arm joint velocities |
| eeRun | 10.0 | Running EE position tracking (chewie_tool0) |
| chewieLiftBarrier | 1000.0 | One-sided barrier: chewie shoulder_lift >= start angle |

## Terminal cost terms

| term | weight | description |
| --- | ---: | --- |
| ee | 1000000.0 | Terminal EE position tracking (chewie_tool0) |
| vStop | 10000.0 | Terminal zero velocity (base + all arm joints) |

## Internal activation weights

### Control regularization (`uReg`)

- w_u (all 16 actuated joints): 0.01

### Base stabilization (`baseRun`, running only)

- w_base[0:3] base position: 100.0
- w_base[nv:nv+3] base linear velocity: 10.0
- w_base[nv+6:] arm joint velocities: 10.0

### Terminal zero velocity (`vStop`)

- w_term[nv:] all velocity components: 100.0

### Chewie lift barrier (`chewieLiftBarrier`)

- lower bound on chewie_shoulder_lift velocity index: 0.0
  (penalizes shoulder_lift dropping below its start angle)

