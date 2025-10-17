# croc_crabs

Minimal setup to optimize and visualize motions for the crab robot using Crocoddyl + Pinocchio + Meshcat.

## Repository layout

- `scripts/` – runnable examples (e.g., `crabs_invdyn.py`, `visualize_crabs.py`).
- `hcrl_crab_robot/` – robot description (URDF + meshes). Managed by Gitman.
- `crocoddyl/` – upstream Crocoddyl sources (read-only for running examples).
- `croc_crabs.yaml` – conda environment.
- `gitman.yaml` – external dependency spec (Gitman).

## 1) Create the environment

```bash
# From repo root
conda env create -f croc_crabs.yaml
conda activate croc_crabs
```

Notes
- The env includes: python 3.9, numpy, matplotlib, pinocchio, crocoddyl, meshcat, and gitman (via pip).

## 2) Fetch external dependencies (Gitman)

This repo uses Gitman to fetch the robot description (and optionally other repos).

```bash
# From repo root
gitman install -c gitman.yaml      # clones into ./hcrl_crab_robot
# (later) lock exact SHAs for reproducibility
gitman lock --all -c gitman.yaml
```

If you update dependencies in `gitman.yaml`, run:
```bash
gitman update -c gitman.yaml
```

## 3) Run an example

`crabs_invdyn.py` solves an inverse-dynamics optimal control problem for the crab robot
and visualizes it in Meshcat and/or plots convergence curves.

- Meshcat (3D viewer):
```bash
python scripts/crabs_invdyn.py display
```
- Plots (matplotlib):
```bash
python scripts/crabs_invdyn.py plot
```
- Both:
```bash
python scripts/crabs_invdyn.py plot display
```

Tips
- Meshcat opens a browser window automatically. If a window doesn’t appear, open the printed URL manually.
- You can also enable via environment variables: `CROCODDYL_DISPLAY=1` and/or `CROCODDYL_PLOT=1`.

## 4) Optional: verify collision geometry in Meshcat

```bash
python scripts/visualize_crabs.py
```

## Troubleshooting

- "ModuleNotFoundError: matplotlib": install it into the active env:
  `conda install -c conda-forge matplotlib`
- Meshcat shows only a single collision object: ensure you’re running the provided scripts (they promote collisions to visuals) and clear the scene between runs.
- If `gitman` is not found, ensure the env is active or install with: `pip install gitman`.

## Git hygiene

When using Gitman, commit only `gitman.yaml` and `gitman.lock`. Ignore vendored folders:
```
hcrl_crab_robot/
crabs/
.gitman/
```
