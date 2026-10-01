# sysid decision log

Newest first. Never edit a block; corrections are new blocks. At milestones, roll old blocks
up into one dated paragraph at the bottom. Times are local (PDT); `~` means the time was
reconstructed from file modification times.

---

## 2026-09-28 ~20:58 · `synth-v2` · synthetic, seed 1, chewie + han
**Question:** Correction to `synth-v1`. What can static poses identify on their own, when only at-rest samples are used?
**Change:** `rows(still_only=True)` for the static run keeps samples with |q̇| < 1e-3 rad/s for ≥ 0.3 s (6625 of 26250).
**Config:** 30 static poses × 2 s, 30 s random Fourier excitation (5 harmonics, f0 = 0.1 Hz, |q̇| ≤ 0.8), 20 s held-out Fourier run. Platform tilt ±5°. Noise: F 0.5 N, T 0.05 N·m, q 2e-5 rad. Bias ~N(0, 3 N / 0.3 N·m), estimated. λ = 1e-3.
**Result (held-out RMS, worst axis):**

| | chewie | han |
|---|---|---|
| noise floor | 0.51 N / 0.52 N·m | 0.68 N / 0.44 N·m |
| URDF | 1.52 N / 5.24 N·m | 1.36 N / 2.47 N·m |
| static only | 0.51 N / 4.47 N·m | 0.68 N / 2.81 N·m |
| static + excitation | 0.51 N / 0.52 N·m | 0.68 N / 0.44 N·m |

- Rank 52 / 70 either way. Condition number: 1.5e5 static only, 3.5e4 with excitation.
- Bias recovered to ≤ 0.07 N.
- Per-body values are not unique:
  - chewie COM errors got *worse* than the URDF's (11–47 mm vs 2–17 mm)
  - han mostly improved (4–18 mm)
  - total mass is right
**Decision:** Treat the per-link output as one physically valid representative, not as measurements. To pin links down: better excitation, and add joint-effort rows.
**Predicted (next run):** Optimizing the excitation (minimize the whitened condition number over the Fourier coefficients) with the same 30 s should bring the condition number from ~3.5e4 to ≤ 5e3 and keep held-out RMS at the noise floor. The rank stays 52, because rank is structural. Per-body COM errors improve somewhat but stay non-unique.
**Artifacts:** `sysid/test_synthetic.py`. Plot only in the scratchpad (`synth_chewie.png`).

## 2026-09-28 ~20:57 · `synth-v1` · synthetic, seed 1, chewie
**Question:** Does the full pipeline (simulate → log like hardware → preprocess → fit) recover a known arm?
**Truth:** URDF + unmodelled masses (0.3 kg on the sensor stack, 0.9 kg upper-arm harness, 0.5 kg forearm harness, 0.4 kg camera on wrist_3) + ±5% mass/inertia and 5 mm COM noise per body.
**Result:**
- Held-out RMS at the noise floor for both "static only" and "static + excitation". URDF: 1.52 N / 5.24 N·m.
- Rank 52 / 70.
**Correction:** "static only" was misleading. The 1.5 s moves between poses leaked dynamic information, so static-only looked as good as with excitation. Superseded by `synth-v2`.

## 2026-09-28 20:49 · `extract-junette_0_3000kg_0` · hardware log, first look
**Question:** What does the hardware log contain, and can it support identification?
**Result:**
- 19.1 s run, 57,887 messages, 11 topics.
- **Joint states:** `/han` and `/chewie/joint_states` have all 6 joints with position, velocity and effort at 500 Hz. The CSV export had truncated han to 2 joints and dropped chewie.
- **Motion:** chewie reaches between ~8 and 13.5 s (J1 3.14 → 0.02 rad at up to 0.94 rad/s, J3 2.60 → 0.96). Han barely moves (J2 −0.12 rad). The platform translates ~7 cm and tilts ~5.3 (degrees, presumably).
- **Wrenches:** `*_dyn_input_wrench` is ~0 at rest before the motion. It is F/T − KDL prediction, tared, not the raw sensor.
- **After the motion:** chewie's residual stays at ≈ (−2.5, −1.3, −1.8) N, (+1.9, …) N·m, i.e. a pose-dependent gravity-compensation error of the order of 0.3 kg. Han's residual shifts ~2 N from the platform tilt alone.
- **During the motion:** chewie's residual reaches ±15–20 N·m, with spikes at the start and end. Suspect latency between F/T and joints, or poor q̈.
- **Clocks:** stamps − log time ≤ 0.1 ms on every topic.
- **Raw F/T:** no raw F/T topic in the log.
**Decision:** Only extract from the `.mcap`. The next recording must include the raw F/T topics and the KDL-predicted wrench.
**Artifacts:** `sysid/extract_mcap.py`, `rss_platform/mcap_data/junette_0_3000kg_0/junette_0_3000kg_0.npz`.

## 2026-09-28 ~20:30 · design decisions (no run)
**Identify from the base F/T sensors, not joint torques.**
- The sensor wrench is linear in the inertial parameters: w = Y(q, q̇, q̈, base motion) π.
- It does not involve joint friction or motor models.
- It is exactly the quantity the hardware's KDL RNE predicts (the modified `ChainIdSolver_RNE` returns `internal_wrenches[0]`).

**Model each arm as a Pinocchio free-flyer rooted at its sensor frame.** The first 6 rows of `computeJointTorqueRegressor` are then the sensor wrench.

**Fit with physical-consistency constraints.** Least squares with every body's pseudo-inertia ⪰ 0, pulled toward the URDF (Wensing et al. 2018). This gives a valid per-link set for the per-link YAML.

**Run the experiments in two stages.** Stage A is static poses, for gravity (quick win for their gravity compensation). Stage B is excitation trajectories, for the full dynamics.

**Noted from the collaborator's KDL mod.** `v[0]` ignores the platform's angular velocity, and `setInducedAcceleration` may mix spatial and point acceleration. The effect is small for slow platforms; quantify it on real data.

---

### Roll-up: 2026-09-28, before sysid
Built the kinematic diagram of `sim_chaser.urdf` (`rss_platform/sim_chaser_diagram.drawio`, in the style of `robot_example.xml`).
- **Page 1:** top and side views with joint axes, link lengths and body masses.
- **Page 2:** per-body mass, COM and inertia with fixed joints merged, plus joint transforms verbatim from the URDF.
- **Findings along the way:**
  - The plate, fingers, cameras, F/T sensors and cable harnesses have no mass in the URDF.
  - Chewie is mounted flipped (its J2/J3/J4/J6 axes point −z).
  - Total system mass is 5153.99 kg (base 5029.28 kg).
