# croc_crabs

Minimal setup to optimize and visualize motions for the crab robot using Crocoddyl + Pinocchio + Meshcat.

## Repository layout

- `scripts/`: runnable examples, e.g. `crabs_invdyn.py`, `visualize_crabs.py` 
- `hcrl_crab_robot/`: robot description (URDF + meshes). Managed by Gitman
- `crocoddyl/`: upstream Crocoddyl sources. Intended to be read-only for learning and examples 
- `croc_crabs.yaml`: conda environment 
- `gitman.yaml`: Gitman-managed external repositories  

## 1. Create the environment

At the base level of the repo, run the following to install all dependencies: 

```bash
conda env create -f croc_crabs.yaml
conda activate croc_crabs
```

## 2. Fetch external repositories with Gitman 

This repo uses Gitman to fetch the robot model (and optionally other repos): 

```bash
# From repo root
gitman update 
```

## 3. Run an example

`crabs_invdyn.py` solves an inverse-dynamics optimal control problem for the crab robot. To just solve the optimization problem and view the output in the terminal, run: 
```bash 
python scripts/crabs_invdyn.py 
```

To visualize the robot motion in Meshcat (3D viewer), run:
```bash
python scripts/crabs_invdyn.py display
```
To view analysis plots, run:
```bash
python scripts/crabs_invdyn.py plot
```
**Warning**: plots are in development and may be incorrect.


To do both:
```bash
python scripts/crabs_invdyn.py plot display
```

Tips
- Meshcat opens a browser window automatically. If a window doesn’t appear, open the printed URL manually. 


## Troubleshooting

- `ModuleNotFoundError: matplotlib`: install it into the active env:
```conda install -c conda-forge matplotlib``` 
- If `gitman` is not found, ensure the env is active or install with `pip install gitman`.
 

