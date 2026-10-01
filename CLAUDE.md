# croc_crabs

## Current state
_Last updated 2026-09-29. Rewrite this block in place at the end of every session._

- **Active task:** system ID of the RSS arms' inertial parameters from base F/T sensors (`sysid/`). Goal: have the pipeline ready before the hardware team finishes calibrating the F/T sensors and the Stewart-platform gravity compensation.
- **Done:** `.mcap` extraction, arm model rooted at the F/T frame, wrench regressor, physically-consistent fit, synthetic end-to-end test.
- **Current config:** 7 bodies × 10 params per arm (B0 = F/T→UR-base stack, B1…B6 = links), estimate sensor bias, λ = 1e-3 prior toward the URDF, Clarabel SDP, 15 Hz zero-phase low-pass on the wrench, Savitzky–Golay derivatives (80 ms window, order 3).
- **Last run:** `synth-v2` (2026-09-28 ~20:58), seed 1, both arms.
- **Best numbers (synthetic, held-out RMS, worst axis):** static poses + 30 s random Fourier excitation reach the noise floor (chewie 0.51 N / 0.52 N·m; URDF 1.52 N / 5.24 N·m). Static poses alone fix forces but not torques during motion (4.47 N·m).
- **Identifiability:** rank 52 / 70, whitened condition number ~3.5e4. Individual link masses/COMs are NOT unique (chewie COM errors up to 47 mm despite a perfect wrench fit).
- **Real data:** 1 run, `junette_0_3000kg_0`. Only chewie moves; han is nearly still; not yet fitted.
- **Blocked on the collaborator:**
  - raw F/T topics (none are recorded yet)
  - which platform pose is the physical one (`/rss/slide_platform_state` or the hexapod `/tf`)
  - the wrench sign convention
  - the KDL chain root
  - what goes into `setInducedAcceleration`
  - the per-link YAML file
- **Next step:** optimized excitation trajectories (minimize the regressor's condition number, within joint and platform limits); then a sanity-check fit of the before/after rest windows of `junette_0_3000kg_0`.

## Working rules
- Read `Current state` above at session start. Read `sysid/LOG.md` only for history on a specific task.
- End of any training/eval session: prepend a block to `sysid/LOG.md`, then update `Current state`.
- Check the newest LOG block's "Predicted" against the new run before analyzing anything else.

## Project
Two UR20 arms (han, chewie) are mounted on a hexapod that emulates a free-floating spacecraft base (hardware-in-the-loop). The hardware subtracts each arm's predicted wrench (KDL RNE, URDF masses from a per-link YAML) from its base F/T reading and feeds the rest to the simulated base. That prediction is wrong because the URDF is missing the camera, cable harnesses and F/T masses. `sysid/` identifies the arms' inertial parameters from F/T data to fix it. Full plan and math: `sysid/README.md`.

Repo map:
- `scripts/`: Crocoddyl planning. `rss_reach.py` is what runs on the hardware.
- `rss_platform/`: a **separate git repo (gitman)** with `sim_chaser.urdf`, the diagram `sim_chaser_diagram.drawio`, and hardware logs `mcap_data/<run>/<run>.mcap`.
- `sysid/`: pipeline `extract_mcap.py` → **[missing: `fit_hardware.py`]** → `arm_model` / `preprocess` / `identify`. `test_synthetic.py` runs the pipeline end to end; its `rows()` is the preprocessing template for hardware.
  README figures come from `sysid/figures/make_figures.py` (uses `drawio_lib.py`; writes the editable `figures.drawio` + one SVG per figure). Edit the script and rerun it; don't hand-edit the SVGs.

## Running
Run from the repo root with the conda env:
```
PY=~/anaconda3/envs/croc_crabs/bin/python        # or: conda activate croc_crabs
$PY sysid/extract_mcap.py rss_platform/mcap_data/junette_0_3000kg_0/junette_0_3000kg_0.mcap
$PY sysid/test_synthetic.py --arm chewie [--seed 1] [--plot out.png]    # ~10 s
```
Scratch outputs (plots) go to the session scratchpad, not the repo.

## Conventions and gotchas
- **Parameters:** `pi` has 10 per body in Pinocchio order `[m, mcx, mcy, mcz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]`, with inertia **about the body-frame origin** (not the COM). Bodies are B0…B6. B_i is moved by J_i, the same numbering as the diagram.
- **Model:** the ArmFT root link is the sensor frame (`han_fts`, `chewie_fts_combined`). Its free-flyer carries the sensor pose/twist; gravity is world −z. Finger prismatic joints are frozen (the fingers are massless in the URDF).
- **Wrench:** `rnea(...)[:6]` is the wrench the **mount applies to the arm**, in the sensor frame. The sensor sign is unknown until confirmed; `rss_reach.py` has `WRENCH_SIGNS = ("mount_on_arm", "arm_on_mount")`.
- **Joint order:** URDF/model order is J1…J6 = shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3. `/joint_states` publishes elbow first; `extract_mcap.py` reorders.
- **Do not use the CSV exports** in `mcap_data/<run>/`. The first message fixes the columns, so `han_joint_states.csv` keeps only 2 joints and has no velocity/effort, and there is no chewie file. Always extract from the `.mcap`.
- **`*_dyn_input_wrench` is not the raw F/T.** It is the F/T minus their KDL arm prediction, tared at the start (~0 N at rest, not ~700 N).
- **`/rss/slide_platform_state`:** x, y, z are probably metres and roll/pitch/yaw probably **degrees** (they reach ±5.3). Unconfirmed. There is no header stamp; use the log time.
- **Clocks:** header stamps agree with the mcap log time to ≤ 0.1 ms, so the logger clock is used for everything.
- **Physical consistency:** use `identify.fit_consistent` (pseudo-inertia ⪰ εI per body), not `fit_ols`, for anything written back to the URDF/YAML. Per-link values are one valid representative of the 52 identifiable combinations, not measurements.
- **Diagram:** the `.drawio` generator script is NOT in the repo; it lived in a session scratchpad. Edit the diagram by hand or ask to recreate the generator.
