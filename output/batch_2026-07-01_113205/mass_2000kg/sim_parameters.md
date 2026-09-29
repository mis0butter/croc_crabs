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

- mass [kg]: 2000.0
- ixx [kg m^2]: 333.3332
- ixy [kg m^2]: 0.0
- ixz [kg m^2]: 0.0
- iyy [kg m^2]: 333.3332
- iyz [kg m^2]: 0.0
- izz [kg m^2]: 333.3332

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

