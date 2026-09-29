# Arm inertial-parameter identification from the base F/T sensors

Identifies the inertial parameters of each RSS arm (han, chewie) from the 6-axis F/T
sensor under it, so that the wrench predicted by the arm model (KDL RNE on the hardware)
matches what the sensor measures. Fixing that prediction is what fixes gravity compensation.

| file | what |
|---|---|
| `extract_mcap.py` | `.mcap` → `.npz` of time series (joint states, wrenches, platform state, `/tf`) |
| `arm_model.py` | Pinocchio model of one arm rooted at its F/T frame; wrench regressor `w = Y(q, v, a) @ pi` |
| `preprocess.py` | zero-phase low-pass, Savitzky-Golay derivatives, resampling |
| `identify.py` | identifiability (SVD), plain LS, and the physically-consistent fit (pseudo-inertia ⪰ 0, pulled toward the URDF) |
| `test_synthetic.py` | end-to-end test on simulated data with known parameters |

Environment: `conda env update -f croc_crabs.yaml` (adds scipy, cvxpy, mcap, mcap-ros2-support).

```
python sysid/extract_mcap.py rss_platform/mcap_data/<run>/<run>.mcap
python sysid/test_synthetic.py --arm chewie --plot synth.png
```

## Parameters

`pi` stacks 10 Pinocchio dynamic parameters per body,
`[m, m·cx, m·cy, m·cz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]` (inertia about the body-frame origin),
for 7 bodies: B0 = everything between the sensor and the UR base (adapter, tombstone,
UR base), B1…B6 = the links moved by J1…J6 (same numbering as the diagram). Fixed-joint links
are merged into the body they hang from (camera, adapters, gripper → B6).

## What the data can and cannot determine

Of the 70 parameters, 52 combinations are identifiable from the F/T wrench; the rest have
no effect on it. Consequences:

- The identified model predicts the sensor wrench to the noise floor (synthetic test).
  That is all KDL needs to compensate the arm correctly.
- Individual link masses/COMs are **not** unique. The fit returns one physically valid
  per-link set (closest to the URDF) that reproduces the identifiable combinations. It is
  fine to write into the per-link YAML for the base-wrench prediction, but do not read
  individual link values as measurements.
- Static poses alone identify gravity (masses × COMs) but not rotational inertia; dynamic
  excitation is needed for the full wrench during motion.
