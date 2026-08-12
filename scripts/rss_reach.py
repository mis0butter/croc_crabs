# ref. https://github.com/PyCQA/pycodestyle/issues/373, remove this for ruff
import argparse
import os
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from datetime import datetime

import matplotlib.pyplot as plt
import meshcat.geometry as g
import meshcat.transformations as tf
import numpy as np
import pinocchio
from pinocchio.visualize import MeshcatVisualizer
from pinocchio import JointModelFreeFlyer

import crocoddyl

# ====================================================================
# Constants
# ====================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(SCRIPT_DIR, "..", "rss_platform")
URDF_PATH = os.path.join(MODEL_DIR, "sim_chaser.urdf")
PACKAGE_DIRS = [MODEL_DIR]

CONTROL_HZ = 50
HORIZON_S = 5.0
CHEWIE_EE_FRAME = "chewie_tool0"
CHEWIE_EE_POS_ENV = np.array([2.0, 0.0, 0.0])

ARM_INIT_POS = {
    # Han: tucked pose for base stabilization
    "han_shoulder_pan_joint": 0.0,
    "han_shoulder_lift_joint": 0.0,
    "han_elbow_joint": -2.6,
    "han_wrist_1_joint": 0.0,
    "han_wrist_2_joint": 0.0,
    "han_wrist_3_joint": 0.0,
    "han_gripper_finger1_joint": 0.0,
    "han_gripper_finger2_joint": 0.0,
    # Chewie: reaching pose
    "chewie_shoulder_pan_joint": 3.14,
    "chewie_shoulder_lift_joint": 3.14,
    "chewie_elbow_joint": 2.6,
    "chewie_wrist_1_joint": 0.0,
    "chewie_wrist_2_joint": 0.0,
    "chewie_wrist_3_joint": 0.0,
    "chewie_gripper_finger1_joint": 0.0,
    "chewie_gripper_finger2_joint": 0.0,
}

HAN_JOINTS = [name for name in ARM_INIT_POS if name.startswith("han_")]
CHEWIE_JOINTS = [name for name in ARM_INIT_POS if name.startswith("chewie_")]

VSTOP_COST_WEIGHT = 1e4
VSTOP_ACTIVATION_WEIGHT = 1e2

# Force/torque sensor mounts. Both sit between the optical plate and the arm tombstone,
# so each carries its whole arm. The URDF has no "chewie_fts"; the resolved Bota LAX
# triplet frame is "chewie_fts_combined". audit_joint is the URDF fixed joint that the
# validation model re-opens so RNEA reports the transmitted wrench directly.
FTS_FRAMES = {
    "han_fts": {
        "audit_joint": "han_fts_joint",
        "hardware": "ATI Omega191",
    },
    "chewie_fts_combined": {
        "audit_joint": "chewie_bota_lax_triplet_mount_to_fts_joint",
        "hardware": "Bota LAX triplet (resolved)",
    },
}

WRENCH_COMPONENTS = ("fx", "fy", "fz", "tx", "ty", "tz")
WRENCH_SIGNS = ("mount_on_arm", "arm_on_mount")
WRENCH_AXES = ("local", "world_aligned")


def _joint_label(joint_name):
    return joint_name.replace("han_", "").replace("chewie_", "").replace("_joint", "")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="RSS Chewie reach trajectory optimization")
    parser.add_argument(
        "modes",
        nargs="*",
        choices=["plot", "display", "video"],
        help="plot: save CSVs/figures; display: Meshcat; video: capture frames",
    )
    parser.add_argument(
        "--mass-kg",
        type=float,
        nargs="+",
        default=None,
        help="armed_mockup mass [kg]; default 5000; pass multiple for batch",
    )
    parser.add_argument(
        "--out-dir",
        type=str,
        default=None,
        help="Output directory (single-mass runs only)",
    )
    parser.add_argument(
        "--batch-dir",
        type=str,
        default=None,
        help="Batch output root; each mass writes to {batch-dir}/mass_{kg}kg/",
    )
    parser.add_argument(
        "--no-validate-wrench",
        dest="validate_wrench",
        action="store_false",
        help="Skip the FTS wrench cross-check against an independently built model",
    )
    args = parser.parse_args(argv)
    if args.mass_kg is None:
        args.mass_kg = [5000.0]
    return args


def _make_output_dir(mass_kg, batch_stamp=None, batch_root=None):
    mass_label = f"{mass_kg:g}".replace(".", "p")
    if batch_root is not None:
        out_dir = os.path.join(batch_root, f"mass_{mass_label}kg")
    else:
        stamp = batch_stamp or datetime.now().strftime("%Y-%m-%d_%H%M%S")
        out_dir = os.path.join(SCRIPT_DIR, "..", "output", f"mass_{mass_label}kg_{stamp}")
    os.makedirs(out_dir, exist_ok=True)
    return out_dir


def _write_csv(path, columns, rows):
    header = ",".join(columns)
    with open(path, "w") as f:
        f.write(header + "\n")
        for row in rows:
            f.write(",".join(f"{val:.9g}" for val in row) + "\n")


def parse_urdf_joint_limits(urdf_path, joint_names):
    limits = {}
    for joint in ET.parse(urdf_path).getroot().findall("joint"):
        name = joint.get("name")
        if name not in joint_names:
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        effort = limit.get("effort")
        limits[name] = {
            "lower": float(limit.get("lower")),
            "upper": float(limit.get("upper")),
            "velocity": float(limit.get("velocity")),
            "effort": None if effort is None else float(effort),
        }
    return limits


def parse_armed_mockup_inertial(urdf_path):
    for link in ET.parse(urdf_path).getroot().findall("link"):
        if link.get("name") != "armed_mockup":
            continue
        inertial = link.find("inertial")
        if inertial is None:
            break
        mass = inertial.find("mass")
        inertia = inertial.find("inertia")
        return {
            "link": "armed_mockup",
            "mass_kg": float(mass.get("value")),
            "ixx": float(inertia.get("ixx")),
            "ixy": float(inertia.get("ixy")),
            "ixz": float(inertia.get("ixz")),
            "iyy": float(inertia.get("iyy")),
            "iyz": float(inertia.get("iyz")),
            "izz": float(inertia.get("izz")),
        }
    return None


def patched_urdf_text(mass_kg=None):
    """Patch sim_chaser.urdf: fix slide joint, optionally scale armed_mockup mass/inertia."""
    urdf_text = open(URDF_PATH).read()
    urdf_text = urdf_text.replace(
        'name="slide_optical_plate_joint" type="floating"',
        'name="slide_optical_plate_joint" type="fixed"',
    )

    if mass_kg is not None:
        baseline = parse_armed_mockup_inertial(URDF_PATH)
        if baseline is None:
            raise RuntimeError("armed_mockup inertial not found in URDF")
        scale = mass_kg / baseline["mass_kg"]
        root = ET.fromstring(urdf_text)
        for link in root.findall("link"):
            if link.get("name") != "armed_mockup":
                continue
            inertial = link.find("inertial")
            mass_el = inertial.find("mass")
            inertia_el = inertial.find("inertia")
            mass_el.set("value", f"{mass_kg:g}")
            for attr in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                inertia_el.set(attr, f"{float(inertia_el.get(attr)) * scale:.9g}")
            break
        urdf_text = ET.tostring(root, encoding="unicode")

    return urdf_text


def patch_urdf_for_load(mass_kg=None):
    with tempfile.NamedTemporaryFile(suffix=".urdf", mode="w", delete=False) as tmp:
        tmp.write(patched_urdf_text(mass_kg))
        return tmp.name


def load_model(mass_kg=None):
    urdf_load_path = patch_urdf_for_load(mass_kg)
    model, collision_model, visual_model = pinocchio.buildModelsFromUrdf(
        urdf_load_path, PACKAGE_DIRS, JointModelFreeFlyer()
    )
    model.gravity.linear[:] = 0.0
    return model, collision_model, visual_model, urdf_load_path


def build_audit_model(mass_kg=None):
    """Same robot, but with the FTS fixed joints re-opened as revolute joints locked at zero.

    Used only to cross-check the wrench: with a real joint at the sensor, RNEA reports the
    transmitted wrench directly in data.f, independent of the frame-tree bookkeeping below.
    """
    root = ET.fromstring(patched_urdf_text(mass_kg))
    audit_joints = {spec["audit_joint"] for spec in FTS_FRAMES.values()}
    found = set()
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name not in audit_joints:
            continue
        joint.set("type", "revolute")
        ET.SubElement(joint, "axis", {"xyz": "0 0 1"})
        ET.SubElement(
            joint,
            "limit",
            {"effort": "0", "lower": "0", "upper": "0", "velocity": "0"},
        )
        found.add(name)
    missing = audit_joints - found
    if missing:
        raise RuntimeError(f"audit joints not found in URDF: {sorted(missing)}")

    model = pinocchio.buildModelFromXML(
        ET.tostring(root, encoding="unicode"), JointModelFreeFlyer()
    )
    model.gravity.linear[:] = 0.0
    return model


def _frame_ancestors(model, frame_id):
    ancestors = []
    current = frame_id
    while True:
        parent = model.frames[current].parentFrame
        if parent == current:
            break
        ancestors.append(parent)
        current = parent
        if current == 0:
            break
    return ancestors


def build_wrench_probe(model, frame_name):
    """Precompute the constant terms needed to cut the model open at a fixed-joint frame.

    Pinocchio merges fixed joints, so both arms hang directly off the free-flyer root joint.
    pinocchio.computeSupportedForceByFrame() sums every direct child joint of that root and
    would report Han's wrench contaminated by Chewie's arm, so the child joints are instead
    filtered by the frame tree.
    """
    frame_id = model.getFrameId(frame_name)
    frame = model.frames[frame_id]
    joint_id = frame.parentJoint

    # Bodies welded to the frame: walk forward through frames whose parentFrame chain
    # already lands inside the set. Their composite inertia never changes.
    welded_frames = {frame_id}
    inertia = frame.placement.act(frame.inertia)
    welded_mass = frame.inertia.mass
    for idx in range(frame_id + 1, model.nframes):
        candidate = model.frames[idx]
        if candidate.parentJoint != joint_id:
            continue
        if candidate.parentFrame not in welded_frames:
            continue
        welded_frames.add(idx)
        inertia += candidate.placement.act(candidate.inertia)
        welded_mass += candidate.inertia.mass

    child_joints = [
        jid
        for jid in range(1, model.njoints)
        if model.parents[jid] == joint_id
        and frame_id in _frame_ancestors(model, model.getFrameId(model.names[jid]))
    ]
    distal_mass = welded_mass + sum(
        model.inertias[sub].mass for jid in child_joints for sub in model.subtrees[jid]
    )

    return {
        "name": frame_name,
        "audit_joint": FTS_FRAMES[frame_name]["audit_joint"],
        "hardware": FTS_FRAMES[frame_name]["hardware"],
        "frame_id": frame_id,
        "joint_id": joint_id,
        "inertia": frame.placement.actInv(inertia),
        "child_joints": child_joints,
        "distal_mass": distal_mass,
        "placement": frame.placement.copy(),
    }


def probe_placement(model, data, probe):
    return data.oMi[probe["joint_id"]] * model.frames[probe["frame_id"]].placement


def frame_subtree_wrench(model, data, probe):
    """Spatial force transmitted from the mount into everything distal of the probe frame.

    Caller must have run pinocchio.forwardKinematics(model, data, q, v, a) and then
    pinocchio.rnea(model, data, q, v, a): rnea leaves data.a untouched, so without the
    forward pass the acceleration term silently evaluates to zero.
    """
    frame = model.frames[probe["frame_id"]]
    inertia = probe["inertia"]
    v = pinocchio.getFrameVelocity(model, data, probe["frame_id"], pinocchio.LOCAL)
    a = pinocchio.getFrameAcceleration(model, data, probe["frame_id"], pinocchio.LOCAL)
    oMf = probe_placement(model, data, probe)

    force = inertia * (a - oMf.actInv(model.gravity)) + v.cross(inertia * v)
    force = frame.placement.act(force)
    for jid in probe["child_joints"]:
        force += data.liMi[jid].act(data.f[jid])
    return frame.placement.actInv(force)


def _initial_configuration(model):
    q0 = pinocchio.neutral(model)
    for joint_name, angle in ARM_INIT_POS.items():
        q0[model.joints[model.getJointId(joint_name)].idx_q] = angle
    return q0


def build_problem(model, q0):
    data = model.createData()
    state = crocoddyl.StateMultibody(model)
    actuation = crocoddyl.ActuationModelFloatingBase(state)
    nu = state.nv

    ee_frame_id = model.getFrameId(CHEWIE_EE_FRAME)
    ee_residual = crocoddyl.ResidualModelFrameTranslation(
        state, ee_frame_id, CHEWIE_EE_POS_ENV, nu
    )
    ee_activation = crocoddyl.ActivationModelQuad(3)
    ee_cost = crocoddyl.CostModelResidual(state, ee_activation, ee_residual)
    ee_run_cost = crocoddyl.CostModelResidual(state, ee_activation, ee_residual)

    w_u = np.ones(actuation.nu) * 0.01
    u_residual = crocoddyl.ResidualModelJointEffort(
        state, actuation, np.zeros(actuation.nu), nu, False
    )
    u_reg_cost = crocoddyl.CostModelResidual(
        state, crocoddyl.ActivationModelWeightedQuad(w_u), u_residual
    )

    x_ref = state.zero()
    x_ref[: model.nq] = q0
    w_base = np.zeros(state.ndx)
    w_base[0:3] = 1e2
    w_base[state.nv : state.nv + 3] = 10
    w_base[state.nv + 6 :] = 10
    base_run_residual = crocoddyl.ResidualModelState(state, x_ref, nu)
    base_run_activation = crocoddyl.ActivationModelWeightedQuad(w_base)
    base_run_cost = crocoddyl.CostModelResidual(
        state, base_run_activation, base_run_residual
    )

    lift_jid = model.getJointId("chewie_shoulder_lift_joint")
    lift_idx_q = model.joints[lift_jid].idx_q
    lift_idx_v = model.joints[lift_jid].idx_v

    x_lift_ref = state.zero()
    x_lift_ref[: model.nq] = q0
    lift_lower = np.full(state.ndx, -np.inf)
    lift_upper = np.full(state.ndx, np.inf)
    lift_lower[lift_idx_v] = 0.0
    lift_barrier_residual = crocoddyl.ResidualModelState(state, x_lift_ref, nu)
    lift_barrier_activation = crocoddyl.ActivationModelQuadraticBarrier(
        crocoddyl.ActivationBounds(lift_lower, lift_upper)
    )
    lift_barrier_cost = crocoddyl.CostModelResidual(
        state, lift_barrier_activation, lift_barrier_residual
    )

    x_term_ref = state.zero()
    w_term = np.zeros(state.ndx)
    w_term[state.nv :] = VSTOP_ACTIVATION_WEIGHT
    vstop_residual = crocoddyl.ResidualModelState(state, x_term_ref, nu)
    vstop_activation = crocoddyl.ActivationModelWeightedQuad(w_term)
    vstop_cost = crocoddyl.CostModelResidual(state, vstop_activation, vstop_residual)

    dt = 1.0 / CONTROL_HZ
    running_cost_model = crocoddyl.CostModelSum(state, nu)
    running_cost_model.addCost("uReg", u_reg_cost, 1 / dt)
    running_cost_model.addCost("baseRun", base_run_cost, 10.0)
    running_cost_model.addCost("eeRun", ee_run_cost, 10.0)
    running_cost_model.addCost("chewieLiftBarrier", lift_barrier_cost, 1e3)

    terminal_cost_model = crocoddyl.CostModelSum(state, nu)
    terminal_cost_model.addCost("ee", ee_cost, 1e6)
    terminal_cost_model.addCost("vStop", vstop_cost, VSTOP_COST_WEIGHT)

    running_model = crocoddyl.IntegratedActionModelEuler(
        crocoddyl.DifferentialActionModelFreeInvDynamics(
            state, actuation, running_cost_model
        ),
        dt,
    )
    terminal_model = crocoddyl.IntegratedActionModelEuler(
        crocoddyl.DifferentialActionModelFreeInvDynamics(
            state, actuation, terminal_cost_model
        ),
        dt,
    )

    T = int(round(HORIZON_S / dt))
    x0 = state.zero()
    x0[: model.nq] = q0
    problem = crocoddyl.ShootingProblem(x0, [running_model] * T, terminal_model)

    sim_params = {
        "control_hz": CONTROL_HZ,
        "dt": dt,
        "horizon_s": HORIZON_S,
        "T": T,
        "gravity": model.gravity.linear.copy(),
        "ee_frame": CHEWIE_EE_FRAME,
        "ee_goal": CHEWIE_EE_POS_ENV,
        "running_costs": [
            {
                "name": "uReg",
                "weight": 1 / dt,
                "description": "Actuation regularization (WeightedQuad, w_u=0.01 per joint)",
            },
            {
                "name": "baseRun",
                "weight": 10.0,
                "description": "Track initial base pose and damp base/arm joint velocities",
            },
            {
                "name": "eeRun",
                "weight": 10.0,
                "description": f"Running EE position tracking ({CHEWIE_EE_FRAME})",
            },
            {
                "name": "chewieLiftBarrier",
                "weight": 1e3,
                "description": "One-sided barrier: chewie shoulder_lift >= start angle",
            },
        ],
        "terminal_costs": [
            {
                "name": "ee",
                "weight": 1e6,
                "description": f"Terminal EE position tracking ({CHEWIE_EE_FRAME})",
            },
            {
                "name": "vStop",
                "weight": VSTOP_COST_WEIGHT,
                "description": "Terminal zero velocity (base + all arm joints)",
            },
        ],
        "w_u": w_u,
        "w_base_position": w_base[0],
        "w_base_lin_vel": w_base[state.nv],
        "w_base_joint_vel": w_base[state.nv + 6],
        "w_term_vel": VSTOP_ACTIVATION_WEIGHT,
        "lift_barrier_lower": lift_lower[lift_idx_v],
    }

    return {
        "data": data,
        "state": state,
        "actuation": actuation,
        "problem": problem,
        "dt": dt,
        "T": T,
        "x0": x0,
        "q0": q0,
        "ee_frame_id": ee_frame_id,
        "lift_idx_q": lift_idx_q,
        "lift_idx_v": lift_idx_v,
        "sim_params": sim_params,
    }


def solve_reach(problem_setup, warm_xs=None, warm_us=None, verbose=True):
    problem = problem_setup["problem"]
    solver = crocoddyl.SolverIntro(problem)
    if verbose:
        solver.setCallbacks([crocoddyl.CallbackVerbose()])

    T = problem_setup["T"]
    x0 = problem_setup["x0"]
    q0 = problem_setup["q0"]
    lift_idx_q = problem_setup["lift_idx_q"]

    if warm_xs is not None and warm_us is not None:
        xs_init = [x.copy() for x in warm_xs]
        us_init = [u.copy() for u in warm_us]
    else:
        xs_init = [x0.copy() for _ in range(T + 1)]
        for k, x in enumerate(xs_init):
            x[lift_idx_q] = q0[lift_idx_q] + 2.0 * (k / T)
        us_init = problem.quasiStatic([x for x in xs_init[:-1]])

    solver.solve(xs_init, us_init)
    return solver


def solution_accelerations(us, nv, n_nodes):
    """DifferentialActionModelFreeInvDynamics uses the control as the joint acceleration
    (nu == state.nv and xout == u), so us[k] is a_k.

    The trajectory has one more state than control, so the terminal node has no command.
    It repeats the last commanded acceleration: vStop makes the terminal deceleration
    large, and zero-filling instead would drop the terminal wrench discontinuously to zero
    and read as a data glitch rather than the zero-order hold it is.
    """
    accelerations = [np.asarray(u, dtype=float).copy() for u in us]
    if not accelerations:
        return [np.zeros(nv) for _ in range(n_nodes)]
    while len(accelerations) < n_nodes:
        accelerations.append(accelerations[-1].copy())
    return accelerations[:n_nodes]


def validate_wrench(model, probes, xs, accelerations, mass_kg):
    """Cross-check frame_subtree_wrench() against a model with real joints at the sensors.

    Returns the largest absolute wrench disagreement over the trajectory, alongside the
    largest wrench magnitude seen, so the caller can report an absolute and relative bound.
    """
    audit = build_audit_model(mass_kg)
    audit_data = audit.createData()
    audit_joints = {p["name"]: audit.getJointId(p["audit_joint"]) for p in probes}
    joint_map = [
        (model.joints[jid], audit.joints[audit.getJointId(model.names[jid])])
        for jid in range(2, model.njoints)
    ]

    data = model.createData()
    q_audit = pinocchio.neutral(audit)
    v_audit = np.zeros(audit.nv)
    a_audit = np.zeros(audit.nv)

    max_error = 0.0
    max_magnitude = 0.0
    for x, a in zip(xs, accelerations):
        q = x[: model.nq]
        v = x[model.nq :]
        pinocchio.forwardKinematics(model, data, q, v, a)
        pinocchio.rnea(model, data, q, v, a)

        q_audit[:7] = q[:7]
        v_audit[:6] = v[:6]
        a_audit[:6] = a[:6]
        for joint, audit_joint in joint_map:
            q_audit[audit_joint.idx_q] = q[joint.idx_q]
            v_audit[audit_joint.idx_v] = v[joint.idx_v]
            a_audit[audit_joint.idx_v] = a[joint.idx_v]
        pinocchio.forwardKinematics(audit, audit_data, q_audit, v_audit, a_audit)
        pinocchio.rnea(audit, audit_data, q_audit, v_audit, a_audit)

        for probe in probes:
            mine = frame_subtree_wrench(model, data, probe)
            reference = audit_data.f[audit_joints[probe["name"]]]
            expected = np.concatenate([reference.linear, reference.angular])
            actual = np.concatenate([mine.linear, mine.angular])
            max_error = max(max_error, np.abs(actual - expected).max())
            max_magnitude = max(max_magnitude, np.abs(expected).max())

    return max_error, max_magnitude


def print_solution_summary(solver, model, data, problem_setup):
    q0 = problem_setup["q0"]
    ee_frame_id = problem_setup["ee_frame_id"]
    lift_idx_q = problem_setup["lift_idx_q"]

    q_final = solver.xs[-1][: model.nq]
    v_final = solver.xs[-1][model.nq :]
    pinocchio.forwardKinematics(model, data, q_final)
    pinocchio.updateFramePlacement(model, data, ee_frame_id)
    ee_final = data.oMf[ee_frame_id].translation.copy()

    print(f"Target {CHEWIE_EE_FRAME} position: {CHEWIE_EE_POS_ENV}")
    print(f"Final   {CHEWIE_EE_FRAME} position: {ee_final}")
    print(f"Position error [m]: {np.linalg.norm(ee_final - CHEWIE_EE_POS_ENV):.4f}")
    print(f"Base position drift [m]: {np.linalg.norm(q_final[:3] - q0[:3]):.4f}")
    print(f"Base orientation drift [quat]: {q_final[3:7] - q0[3:7]}")
    print(f"Final velocity norm: {np.linalg.norm(v_final):.6e}")

    lift_traj = np.array([x[lift_idx_q] for x in solver.xs])
    print(
        f"chewie shoulder_lift: start={lift_traj[0]:.3f} "
        f"min={lift_traj.min():.3f} max={lift_traj.max():.3f} end={lift_traj[-1]:.3f}"
    )


def _extract_trajectory_data(xs, us, model, data, dt, ee_frame_id, ee_goal, probes):
    times = np.arange(len(xs)) * dt
    accelerations = solution_accelerations(us, model.nv, len(xs))
    positions = {name: [] for name in ARM_INIT_POS}
    velocities = {name: [] for name in ARM_INIT_POS}
    torques = {name: [] for name in ARM_INIT_POS}
    wrenches = {probe["name"]: {axes: [] for axes in WRENCH_AXES} for probe in probes}
    base_residual = []
    ee_errors = []
    ee_pos = []
    base_rpy = []
    base_omega = []
    base_pos = []
    base_lin_vel = []
    com_pos = []

    for x, a in zip(xs, accelerations):
        q = x[: model.nq]
        v = x[model.nq :]

        pinocchio.centerOfMass(model, data, q)
        pinocchio.forwardKinematics(model, data, q, v, a)
        tau = pinocchio.rnea(model, data, q, v, a)
        pinocchio.updateFramePlacement(model, data, ee_frame_id)

        for joint_name in ARM_INIT_POS:
            jid = model.getJointId(joint_name)
            positions[joint_name].append(q[model.joints[jid].idx_q])
            velocities[joint_name].append(v[model.joints[jid].idx_v])
            torques[joint_name].append(tau[model.joints[jid].idx_v])
        # The floating base is unactuated, so these six entries are a residual the solver
        # already constrains to zero; carrying them out makes that check auditable.
        base_residual.append(tau[:6].copy())

        for probe in probes:
            force = frame_subtree_wrench(model, data, probe)
            rotation = probe_placement(model, data, probe).rotation
            wrenches[probe["name"]]["local"].append(
                np.concatenate([force.linear, force.angular])
            )
            wrenches[probe["name"]]["world_aligned"].append(
                np.concatenate([rotation @ force.linear, rotation @ force.angular])
            )

        ee_position = data.oMf[ee_frame_id].translation.copy()
        ee_errors.append(np.linalg.norm(ee_position - ee_goal))
        ee_pos.append(ee_position)

        quat = pinocchio.Quaternion(q[6], q[3], q[4], q[5])
        base_rpy.append(pinocchio.rpy.matrixToRpy(quat.toRotationMatrix()))
        base_omega.append(v[3:6].copy())
        base_pos.append(q[0:3].copy())
        base_lin_vel.append(v[0:3].copy())
        com_pos.append(data.com[0].copy())

    return {
        "times": times,
        "positions": {name: np.asarray(vals) for name, vals in positions.items()},
        "velocities": {name: np.asarray(vals) for name, vals in velocities.items()},
        "torques": {name: np.asarray(vals) for name, vals in torques.items()},
        "wrenches": {
            name: {axes: np.asarray(vals) for axes, vals in per_axes.items()}
            for name, per_axes in wrenches.items()
        },
        "base_residual": np.asarray(base_residual),
        "n_controls": len(us),
        "ee_errors": np.asarray(ee_errors),
        "ee_pos": np.asarray(ee_pos),
        "base_rpy": np.asarray(base_rpy),
        "base_omega": np.asarray(base_omega),
        "base_pos": np.asarray(base_pos),
        "base_lin_vel": np.asarray(base_lin_vel),
        "com_pos": np.asarray(com_pos),
    }


def _csv_cell(value):
    text = np.array2string(value, separator=" ") if isinstance(value, np.ndarray) else str(value)
    if any(c in text for c in [",", '"', "\n"]):
        return '"' + text.replace('"', '""') + '"'
    return text


def _save_sim_parameters(out_dir, urdf_path, sim_params):
    inertial = parse_armed_mockup_inertial(urdf_path)
    md_path = os.path.join(out_dir, "sim_parameters.md")
    csv_path = os.path.join(out_dir, "sim_parameters.csv")

    lines = [
        "# RSS reach simulation parameters",
        "",
        "## Problem setup",
        "",
        f"- control_hz: {sim_params['control_hz']}",
        f"- dt [s]: {sim_params['dt']}",
        f"- horizon_s [s]: {sim_params['horizon_s']}",
        f"- T (nodes): {sim_params['T']}",
        f"- gravity [m/s^2]: {np.array2string(sim_params['gravity'], separator=' ')}",
        f"- ee_frame: {sim_params['ee_frame']}",
        f"- ee_goal [m]: {np.array2string(sim_params['ee_goal'], separator=' ')}",
        "",
        "## Armed mockup inertial (`armed_mockup` link, URDF)",
        "",
    ]
    csv_rows = [
        ("section", "parameter", "value"),
        ("problem", "control_hz", sim_params["control_hz"]),
        ("problem", "dt_s", sim_params["dt"]),
        ("problem", "horizon_s", sim_params["horizon_s"]),
        ("problem", "T", sim_params["T"]),
        ("problem", "gravity_m_s2", sim_params["gravity"]),
        ("problem", "ee_frame", sim_params["ee_frame"]),
        ("problem", "ee_goal_m", sim_params["ee_goal"]),
    ]

    if inertial:
        lines.extend(
            [
                f"- mass [kg]: {inertial['mass_kg']}",
                f"- ixx [kg m^2]: {inertial['ixx']}",
                f"- ixy [kg m^2]: {inertial['ixy']}",
                f"- ixz [kg m^2]: {inertial['ixz']}",
                f"- iyy [kg m^2]: {inertial['iyy']}",
                f"- iyz [kg m^2]: {inertial['iyz']}",
                f"- izz [kg m^2]: {inertial['izz']}",
                "",
            ]
        )
        for key in ("mass_kg", "ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            csv_rows.append(("armed_mockup_inertial", key, inertial[key]))
    else:
        lines.append("- armed_mockup inertial not found in URDF")
        lines.append("")

    lines.extend(_fts_convention_lines(sim_params))
    for probe in sim_params.get("fts_probes", []):
        csv_rows.append(("fts_frame", f"{probe['name']}_distal_mass_kg", probe["distal_mass"]))
        csv_rows.append(
            ("fts_frame", f"{probe['name']}_origin_m", probe["origin_in_base"])
        )
    if "base_residual_max" in sim_params:
        csv_rows.append(
            ("validation", "base_residual_max", sim_params["base_residual_max"])
        )
    if sim_params.get("wrench_audit_max_error") is not None:
        csv_rows.append(
            ("validation", "wrench_audit_max_error", sim_params["wrench_audit_max_error"])
        )
        csv_rows.append(
            ("validation", "wrench_max_magnitude", sim_params["wrench_max_magnitude"])
        )

    lines.extend(
        [
            "## Running cost terms",
            "",
            "| term | weight | description |",
            "| --- | ---: | --- |",
        ]
    )
    for cost in sim_params["running_costs"]:
        lines.append(f"| {cost['name']} | {cost['weight']} | {cost['description']} |")
        csv_rows.append(("running_cost", cost["name"], cost["weight"]))
        csv_rows.append(("running_cost_description", cost["name"], cost["description"]))

    lines.extend(
        [
            "",
            "## Terminal cost terms",
            "",
            "| term | weight | description |",
            "| --- | ---: | --- |",
        ]
    )
    for cost in sim_params["terminal_costs"]:
        lines.append(f"| {cost['name']} | {cost['weight']} | {cost['description']} |")
        csv_rows.append(("terminal_cost", cost["name"], cost["weight"]))
        csv_rows.append(("terminal_cost_description", cost["name"], cost["description"]))

    lines.extend(
        [
            "",
            "## Internal activation weights",
            "",
            "### Control regularization (`uReg`)",
            "",
            f"- w_u (all {len(sim_params['w_u'])} actuated joints): {sim_params['w_u'][0]}",
            "",
            "### Base stabilization (`baseRun`, running only)",
            "",
            f"- w_base[0:3] base position: {sim_params['w_base_position']}",
            f"- w_base[nv:nv+3] base linear velocity: {sim_params['w_base_lin_vel']}",
            f"- w_base[nv+6:] arm joint velocities: {sim_params['w_base_joint_vel']}",
            "",
            "### Terminal zero velocity (`vStop`)",
            "",
            f"- w_term[nv:] all velocity components: {sim_params['w_term_vel']}",
            "",
            "### Chewie lift barrier (`chewieLiftBarrier`)",
            "",
            f"- lower bound on chewie_shoulder_lift velocity index: {sim_params['lift_barrier_lower']}",
            "  (penalizes shoulder_lift dropping below its start angle)",
            "",
        ]
    )
    csv_rows.extend(
        [
            ("activation", "w_u", sim_params["w_u"][0]),
            ("activation", "w_base_position", sim_params["w_base_position"]),
            ("activation", "w_base_lin_vel", sim_params["w_base_lin_vel"]),
            ("activation", "w_base_joint_vel", sim_params["w_base_joint_vel"]),
            ("activation", "w_term_vel", sim_params["w_term_vel"]),
            ("activation", "lift_barrier_lower", sim_params["lift_barrier_lower"]),
        ]
    )

    with open(md_path, "w") as f:
        f.write("\n".join(lines) + "\n")

    with open(csv_path, "w") as f:
        f.write("section,parameter,value\n")
        for section, parameter, value in csv_rows[1:]:
            f.write(f"{section},{parameter},{_csv_cell(value)}\n")

    print(f"Saved sim parameters to {os.path.abspath(md_path)}")


def _fts_convention_lines(sim_params):
    probes = sim_params.get("fts_probes", [])
    lines = [
        "## Force/torque sensor wrench",
        "",
        "Each sensor frame sits between the optical plate and the arm tombstone, so it",
        "carries the whole arm above it. Gravity is zero in the model, so the reported",
        "wrench is purely motion-induced and is directly comparable to a",
        "gravity-compensated hardware measurement. The final sample repeats the last",
        "commanded acceleration, since there is one more state than control.",
        "",
        "| frame | hardware | distal mass [kg] | origin in base frame [m] |",
        "| --- | --- | ---: | --- |",
    ]
    for probe in probes:
        origin = np.array2string(probe["origin_in_base"], separator=" ")
        lines.append(
            f"| `{probe['name']}` | {probe['hardware']} | "
            f"{probe['distal_mass']:.4f} | {origin} |"
        )

    lines.extend(
        [
            "",
            "### Column conventions in `<frame>_wrench.csv`",
            "",
            "- Torques are taken about the origin of the named frame.",
            "- `local`: components in the sensor frame's own axes.",
            "- `world_aligned`: same origin, components rotated into the inertial frame.",
            "- `mount_on_arm`: force the mount/table side applies to the arm assembly",
            "  (Pinocchio/RNEA convention).",
            "- `arm_on_mount`: the reaction, i.e. what the arm pushes into the sensor.",
            "  Equal to `-mount_on_arm`.",
            "",
            "### Validation",
            "",
        ]
    )
    if "base_residual_max" in sim_params:
        lines.append(
            "- max |unactuated floating-base residual| over the commanded nodes: "
            f"{sim_params['base_residual_max']:.3e} (N and N m; should be ~0). The"
            " terminal node in `validation.csv` is flagged `commanded=0` because it"
            " extrapolates the last command and is not constrained by the solver."
        )
    if sim_params.get("wrench_audit_max_error") is not None:
        lines.append(
            "- max wrench disagreement against an independent model with real joints at "
            f"the sensors: {sim_params['wrench_audit_max_error']:.3e} "
            f"against a peak magnitude of {sim_params['wrench_max_magnitude']:.3f}"
        )
    lines.append("")
    return lines


def _plot_joint_limits(ax, joint_name, limits, kind):
    if joint_name not in limits:
        return
    if kind == "position":
        ax.axhline(
            limits[joint_name]["lower"],
            color="gray",
            ls="--",
            lw=1,
            alpha=0.8,
            label="limit",
        )
        ax.axhline(
            limits[joint_name]["upper"],
            color="gray",
            ls="--",
            lw=1,
            alpha=0.8,
        )
    else:
        bound = limits[joint_name]["velocity" if kind == "velocity" else "effort"]
        if bound is None:
            return
        ax.axhline(bound, color="gray", ls="--", lw=1, alpha=0.8, label="limit")
        ax.axhline(-bound, color="gray", ls="--", lw=1, alpha=0.8)


def _save_arm_joint_figure(out_dir, traj, limits, arm_name, joint_names, color, linestyle):
    times = traj["times"]
    positions = traj["positions"]
    velocities = traj["velocities"]

    fig = plt.figure(figsize=(14, 11))
    gs = fig.add_gridspec(
        5,
        4,
        height_ratios=[1, 1, 0.1, 1, 1],
        hspace=0.4,
        wspace=0.25,
        top=0.93,
        bottom=0.06,
        left=0.06,
        right=0.98,
    )

    torques = traj["torques"]

    csv_columns = ["time_s"]
    csv_rows = []
    for k, time_s in enumerate(times):
        row = [time_s]
        for joint_name in joint_names:
            row.extend(
                [
                    positions[joint_name][k],
                    velocities[joint_name][k],
                    torques[joint_name][k],
                ]
            )
        csv_rows.append(row)

    for joint_idx, joint_name in enumerate(joint_names):
        label = _joint_label(joint_name)
        csv_columns.extend([f"{label}_pos", f"{label}_vel", f"{label}_tau"])

        pos_row, pos_col = divmod(joint_idx, 4)
        pos_ax = fig.add_subplot(gs[pos_row, pos_col])
        vel_ax = fig.add_subplot(gs[pos_row + 3, pos_col], sharex=pos_ax)

        pos_ax.plot(times, positions[joint_name], color=color, ls=linestyle, label=arm_name)
        _plot_joint_limits(pos_ax, joint_name, limits, "position")
        pos_ax.set_title(label)
        pos_ax.grid(True, alpha=0.3)

        vel_ax.plot(times, velocities[joint_name], color=color, ls=linestyle)
        _plot_joint_limits(vel_ax, joint_name, limits, "velocity")
        vel_ax.set_title(label)
        vel_ax.grid(True, alpha=0.3)

        if joint_idx == 0:
            pos_ax.legend(loc="upper right", fontsize=7)
        if pos_row == 0:
            plt.setp(pos_ax.get_xticklabels(), visible=False)
        else:
            pos_ax.set_xlabel("time [s]")
        if pos_row == 0:
            plt.setp(vel_ax.get_xticklabels(), visible=False)
        else:
            vel_ax.set_xlabel("time [s]")

    fig.text(
        0.5,
        0.975,
        f"{arm_name} joint positions [rad]",
        ha="center",
        fontsize=15,
        fontweight="bold",
    )
    fig.text(
        0.5,
        0.48,
        f"{arm_name} joint velocities [rad/s]",
        ha="center",
        fontsize=15,
        fontweight="bold",
    )
    fig.savefig(
        os.path.join(out_dir, f"{arm_name}_joints.png"), dpi=150, bbox_inches="tight"
    )
    _write_csv(os.path.join(out_dir, f"{arm_name}_joints.csv"), csv_columns, csv_rows)
    plt.close(fig)


def _save_arm_torque_figure(out_dir, traj, limits, arm_name, joint_names, color):
    times = traj["times"]
    torques = traj["torques"]

    fig, axes = plt.subplots(2, 4, figsize=(16, 7), sharex=True)
    for joint_idx, joint_name in enumerate(joint_names):
        ax = axes[divmod(joint_idx, 4)]
        label = _joint_label(joint_name)
        ax.plot(times, torques[joint_name], color=color, label=arm_name)

        peak = float(np.abs(torques[joint_name]).max())
        effort = limits.get(joint_name, {}).get("effort")
        title = label
        if effort:
            # In zero gravity the efforts are orders of magnitude below the UR20 limits, so
            # drawing the limit lines would flatten the trace; report utilization instead.
            title = f"{label}\npeak {peak:.2f}, {100.0 * peak / effort:.1f}% of limit"
            if peak > 0.25 * effort:
                _plot_joint_limits(ax, joint_name, limits, "effort")
        ax.set_title(title, fontsize=10)
        ax.grid(True, alpha=0.3)
        if joint_idx >= 4:
            ax.set_xlabel("time [s]")
        if joint_idx == 0:
            ax.legend(loc="upper right", fontsize=7)

    fig.suptitle(
        f"{arm_name} joint effort (N m for revolute joints, N for the prismatic fingers)",
        fontsize=14,
        fontweight="bold",
    )
    fig.tight_layout()
    fig.savefig(
        os.path.join(out_dir, f"{arm_name}_torques.png"), dpi=150, bbox_inches="tight"
    )
    plt.close(fig)


def _save_base_figure(out_dir, traj, q_ref):
    times = traj["times"]
    rpy_ref = pinocchio.rpy.matrixToRpy(
        pinocchio.Quaternion(q_ref[6], q_ref[3], q_ref[4], q_ref[5]).toRotationMatrix()
    )

    csv_columns = [
        "time_s",
        "base_x",
        "base_y",
        "base_z",
        "base_vx",
        "base_vy",
        "base_vz",
        "base_roll",
        "base_pitch",
        "base_yaw",
        "base_wx",
        "base_wy",
        "base_wz",
    ]
    csv_rows = [
        [
            t,
            *traj["base_pos"][k],
            *traj["base_lin_vel"][k],
            *traj["base_rpy"][k],
            *traj["base_omega"][k],
        ]
        for k, t in enumerate(times)
    ]

    fig, axes = plt.subplots(2, 2, figsize=(12, 7))

    pos_labels = ["x", "y", "z"]
    for i, label in enumerate(pos_labels):
        axes[0, 0].plot(times, traj["base_pos"][:, i], label=label)
        axes[0, 0].axhline(q_ref[i], color=f"C{i}", ls=":", lw=1, alpha=0.7)
    axes[0, 0].set_ylabel("position [m]")
    axes[0, 0].set_title("Base position")
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend(ncol=3, fontsize=8)

    for i, label in enumerate(pos_labels):
        axes[0, 1].plot(times, traj["base_lin_vel"][:, i], label=f"v{label}")
    axes[0, 1].set_ylabel("velocity [m/s]")
    axes[0, 1].set_title("Base linear velocity")
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend(ncol=3, fontsize=8)

    rpy_labels = ["roll", "pitch", "yaw"]
    for i, label in enumerate(rpy_labels):
        axes[1, 0].plot(times, traj["base_rpy"][:, i], label=label)
        axes[1, 0].axhline(rpy_ref[i], color=f"C{i}", ls=":", lw=1, alpha=0.7)
    axes[1, 0].set_xlabel("time [s]")
    axes[1, 0].set_ylabel("orientation [rad]")
    axes[1, 0].set_title("Base orientation (RPY)")
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend(ncol=3, fontsize=8)

    omega_labels = ["wx", "wy", "wz"]
    for i, label in enumerate(omega_labels):
        axes[1, 1].plot(times, traj["base_omega"][:, i], label=label)
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].set_ylabel("angular velocity [rad/s]")
    axes[1, 1].set_title("Base angular velocity")
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend(ncol=3, fontsize=8)

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "base_motion.png"), dpi=150, bbox_inches="tight")
    _write_csv(os.path.join(out_dir, "base_motion.csv"), csv_columns, csv_rows)
    plt.close(fig)


def _save_com_ee_figure(out_dir, traj, ee_goal, ee_frame_name):
    times = traj["times"]
    com = traj["com_pos"]
    ee = traj["ee_pos"]

    csv_columns = [
        "time_s",
        "com_x",
        "com_y",
        "com_z",
        "ee_x",
        "ee_y",
        "ee_z",
        "ee_error",
    ]
    csv_rows = [
        [t, *com[k], *ee[k], traj["ee_errors"][k]]
        for k, t in enumerate(times)
    ]

    fig = plt.figure(figsize=(12, 9))
    gs = fig.add_gridspec(3, 2, width_ratios=[1, 1.2], hspace=0.35, wspace=0.3)

    ax_com_x = fig.add_subplot(gs[0, 0])
    ax_com_y = fig.add_subplot(gs[1, 0], sharex=ax_com_x)
    ax_com_z = fig.add_subplot(gs[2, 0], sharex=ax_com_x)
    ax_3d = fig.add_subplot(gs[0:2, 1], projection="3d")
    ax_err = fig.add_subplot(gs[2, 1], sharex=ax_com_x)

    ax_com_x.plot(times, com[:, 0], color="tab:green")
    ax_com_x.set_ylabel("x [m]")
    ax_com_x.set_title("COM x")
    ax_com_x.grid(True, alpha=0.3)

    ax_com_y.plot(times, com[:, 1], color="tab:green")
    ax_com_y.set_ylabel("y [m]")
    ax_com_y.set_title("COM y")
    ax_com_y.grid(True, alpha=0.3)

    ax_com_z.plot(times, com[:, 2], color="tab:green")
    ax_com_z.set_xlabel("time [s]")
    ax_com_z.set_ylabel("z [m]")
    ax_com_z.set_title("COM z")
    ax_com_z.grid(True, alpha=0.3)

    plt.setp(ax_com_x.get_xticklabels(), visible=False)
    plt.setp(ax_com_y.get_xticklabels(), visible=False)

    ax_err.plot(
        times,
        traj["ee_errors"],
        color="tab:purple",
        label=f"||p_ee - p_goal|| ({ee_frame_name})",
    )
    ax_err.set_xlabel("time [s]")
    ax_err.set_ylabel("error [m]")
    ax_err.set_title("End-effector tracking error")
    ax_err.grid(True, alpha=0.3)
    ax_err.legend(fontsize=8)

    ax_3d.scatter(
        com[:, 0],
        com[:, 1],
        com[:, 2],
        color="tab:green",
        marker="o",
        s=40,
        label="COM",
    )
    ax_3d.plot(ee[:, 0], ee[:, 1], ee[:, 2], color="tab:purple", label="EE")
    ax_3d.scatter(
        ee_goal[0],
        ee_goal[1],
        ee_goal[2],
        color="gold",
        marker="*",
        s=120,
        label="EE goal",
    )
    ax_3d.set_xlabel("x [m]")
    ax_3d.set_ylabel("y [m]")
    ax_3d.set_zlabel("z [m]")
    ax_3d.set_title("COM and end-effector position")
    ax_3d.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "com_ee_tracking.png"), dpi=150, bbox_inches="tight")
    _write_csv(os.path.join(out_dir, "com_ee_tracking.csv"), csv_columns, csv_rows)
    plt.close(fig)


def _wrench_signed(traj, frame_name, sign, axes):
    wrench = traj["wrenches"][frame_name][axes]
    return wrench if sign == "mount_on_arm" else -wrench


def _save_fts_wrench_outputs(out_dir, traj, probes):
    times = traj["times"]

    csv_columns = ["time_s"]
    for sign in WRENCH_SIGNS:
        for axes in WRENCH_AXES:
            csv_columns.extend(f"{sign}_{axes}_{comp}" for comp in WRENCH_COMPONENTS)

    for probe in probes:
        blocks = [
            _wrench_signed(traj, probe["name"], sign, axes)
            for sign in WRENCH_SIGNS
            for axes in WRENCH_AXES
        ]
        csv_rows = [
            [time_s, *(value for block in blocks for value in block[k])]
            for k, time_s in enumerate(times)
        ]
        _write_csv(
            os.path.join(out_dir, f"{probe['name']}_wrench.csv"), csv_columns, csv_rows
        )

    fig, axes_grid = plt.subplots(
        2, len(probes), figsize=(7 * len(probes), 8), sharex=True, squeeze=False
    )
    for col, probe in enumerate(probes):
        wrench = _wrench_signed(traj, probe["name"], "arm_on_mount", "local")
        force_ax = axes_grid[0, col]
        torque_ax = axes_grid[1, col]
        for i, comp in enumerate(("fx", "fy", "fz")):
            force_ax.plot(times, wrench[:, i], label=comp)
        for i, comp in enumerate(("tx", "ty", "tz")):
            torque_ax.plot(times, wrench[:, 3 + i], label=comp)
        force_ax.set_title(
            f"{probe['name']} force [N]\n{probe['hardware']}, "
            f"{probe['distal_mass']:.1f} kg distal"
        )
        force_ax.set_ylabel("force [N]")
        torque_ax.set_title(f"{probe['name']} torque [N m]")
        torque_ax.set_ylabel("torque [N m]")
        torque_ax.set_xlabel("time [s]")
        for ax in (force_ax, torque_ax):
            ax.grid(True, alpha=0.3)
            ax.legend(ncol=3, fontsize=8)

    fig.suptitle(
        "Motion-induced wrench at the FTS mounts (arm on mount, sensor frame axes, zero gravity)",
        fontsize=14,
        fontweight="bold",
    )
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fts_wrench.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


def _save_validation_csv(out_dir, traj):
    residual = traj["base_residual"]
    n_controls = traj["n_controls"]
    columns = (
        ["time_s"]
        + [f"base_residual_{comp}" for comp in WRENCH_COMPONENTS]
        + ["commanded"]
    )
    rows = [
        [t, *residual[k], 1 if k < n_controls else 0]
        for k, t in enumerate(traj["times"])
    ]
    _write_csv(os.path.join(out_dir, "validation.csv"), columns, rows)
    # Only the commanded nodes carry the solver's unactuated-base equality constraint; the
    # terminal node extrapolates the last command, so its residual is meaningless here.
    return np.abs(residual[:n_controls]).max()


def save_rss_reach_outputs(
    xs,
    us,
    model,
    data,
    dt,
    ee_frame_id,
    ee_goal,
    q_ref,
    urdf_path,
    out_dir,
    sim_params,
    probes,
):
    traj = _extract_trajectory_data(
        xs, us, model, data, dt, ee_frame_id, ee_goal, probes
    )
    limits = parse_urdf_joint_limits(URDF_PATH, list(ARM_INIT_POS.keys()))
    _save_arm_joint_figure(out_dir, traj, limits, "han", HAN_JOINTS, "tab:blue", "-")
    _save_arm_joint_figure(
        out_dir, traj, limits, "chewie", CHEWIE_JOINTS, "tab:red", "-"
    )
    _save_arm_torque_figure(out_dir, traj, limits, "han", HAN_JOINTS, "tab:blue")
    _save_arm_torque_figure(out_dir, traj, limits, "chewie", CHEWIE_JOINTS, "tab:red")
    _save_base_figure(out_dir, traj, q_ref)
    _save_com_ee_figure(out_dir, traj, ee_goal, sim_params["ee_frame"])
    _save_fts_wrench_outputs(out_dir, traj, probes)
    sim_params["base_residual_max"] = _save_validation_csv(out_dir, traj)
    _save_sim_parameters(out_dir, urdf_path, sim_params)
    print(f"Saved figures and CSVs to {os.path.abspath(out_dir)}")


def _init_ee_goal_marker(viz, goal_pos, radius):
    viz.viewer["chewie_ee_goal"].set_object(
        g.Sphere(radius),
        g.MeshLambertMaterial(color=0xFF0000),
    )
    viz.viewer["chewie_ee_goal"].set_transform(tf.translation_matrix(goal_pos))


def _encode_video(frame_dir, mp4_path, fps):
    cmd = [
        "ffmpeg",
        "-y",
        "-framerate",
        f"{fps:.0f}",
        "-i",
        os.path.join(frame_dir, "frame_%06d.png"),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        mp4_path,
    ]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    print(f"Saved video to {os.path.abspath(mp4_path)}")


def display_motion(solver, model, collision_model, visual_model, out_dir, dt, with_display, with_video):
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    # captureImage() needs a live Meshcat viewer; open browser for video even without display loop.
    viz.initViewer(open=with_display or with_video)
    viz.loadViewerModel()
    _init_ee_goal_marker(viz, CHEWIE_EE_POS_ENV, 0.05)

    qs = [x[: model.nq] for x in solver.xs]
    fps = 1.0 / dt

    if with_video:
        frame_dir = os.path.join(out_dir, "frames")
        os.makedirs(frame_dir, exist_ok=True)
        n_frames = len(qs)
        print(f"Capturing {n_frames} frames to {os.path.abspath(frame_dir)}")
        for frame_idx, q in enumerate(qs):
            viz.display(q)
            plt.imsave(
                os.path.join(frame_dir, f"frame_{frame_idx:06d}.png"),
                viz.captureImage(),
            )
            if frame_idx % 50 == 0 or frame_idx == n_frames - 1:
                print(f"  frame {frame_idx + 1}/{n_frames}")
        mp4_path = os.path.join(out_dir, "rss_reach.mp4")
        _encode_video(frame_dir, mp4_path, fps)

    if not with_display:
        return

    pause = 2.0
    while True:
        for q in qs:
            viz.display(q)
            time.sleep(dt)
        time.sleep(pause)


def write_batch_readme(batch_root, mass_dirs):
    readme_path = os.path.join(batch_root, "README_deliverable.md")
    lines = [
        "# RSS reach trajectories — mass sweep",
        "",
        "Generated by `scripts/rss_reach.py`.",
        "",
        "## Mass variants",
        "",
        "Each folder uses the same start pose and Chewie EE goal `[2, 0, 0]` m.",
        "`armed_mockup` inertia is scaled linearly from the 5000 kg box baseline.",
        "Terminal cost `vStop` drives all velocities to zero at the final timestep.",
        "",
        "## Files per mass folder",
        "",
        "| File | Description |",
        "| --- | --- |",
        "| `han_joints.csv` | Han arm joint position/velocity/effort commands [rad, rad/s, N m] |",
        "| `chewie_joints.csv` | Chewie arm joint position/velocity/effort commands |",
        "| `han_fts_wrench.csv` | Predicted wrench at `han_fts` (ATI Omega191 mount) |",
        "| `chewie_fts_combined_wrench.csv` | Predicted wrench at `chewie_fts_combined` (Bota LAX triplet) |",
        "| `base_motion.csv` | Expected platform position, linear vel, RPY, angular vel |",
        "| `com_ee_tracking.csv` | COM, EE position, EE tracking error |",
        "| `validation.csv` | Unactuated floating-base residual per timestep (~0 where `commanded=1`) |",
        "| `sim_parameters.md` | Problem setup, mass/inertia, cost weights, wrench conventions |",
        "| `fts_wrench.png`, `*_torques.png` | Wrench and joint effort plots |",
        "| `rss_reach.mp4` | Meshcat animation (when run with `video`) |",
        "",
        "## Wrench conventions",
        "",
        "`han_fts` and `chewie_fts_combined` sit between the optical plate and the arm",
        "tombstone, so each carries its entire arm — this is what the mount force/torque",
        "sensors physically measure. The URDF has no `chewie_fts` link; the resolved Bota",
        "LAX triplet frame is `chewie_fts_combined`.",
        "",
        "Gravity is zero in the model, so these wrenches are purely motion-induced and",
        "line up with a gravity-compensated hardware measurement.",
        "",
        "Every `*_wrench.csv` carries the same wrench four ways, so no sign or frame",
        "convention has to be guessed:",
        "",
        "- `mount_on_arm_*`: force the mount/table applies to the arm assembly.",
        "- `arm_on_mount_*`: the reaction, what the arm pushes into the sensor (`= -mount_on_arm`).",
        "- `*_local_*`: components in the sensor frame's own axes.",
        "- `*_world_aligned_*`: same origin, components rotated into the inertial frame.",
        "",
        "Torques are taken about the sensor frame origin. The trajectory has one more state",
        "than control, so the final row repeats the last commanded acceleration.",
        "",
        "## Folders",
        "",
    ]
    for mass_kg, out_dir in mass_dirs:
        rel = os.path.relpath(out_dir, batch_root)
        lines.append(f"- `{rel}/` — {mass_kg:g} kg")

    masses = " ".join(f"{mass_kg:g}" for mass_kg, _ in mass_dirs)
    lines.extend(
        [
            "",
            "## Regenerating",
            "",
            "```bash",
            f"python scripts/rss_reach.py plot --mass-kg {masses} \\",
            f"    --batch-dir {os.path.basename(batch_root)}",
            "```",
            "",
            "Add `video` to also render a Meshcat animation per mass.",
            "",
        ]
    )
    with open(readme_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Wrote {os.path.abspath(readme_path)}")


def run_single_mass(
    mass_kg,
    with_plot,
    with_display,
    with_video,
    out_dir,
    warm_xs=None,
    warm_us=None,
    verbose=True,
    validate_wrench_flag=True,
):
    print(f"\n{'=' * 60}")
    print(f"Solving reach trajectory: armed_mockup mass = {mass_kg:g} kg")
    print(f"{'=' * 60}")

    model, collision_model, visual_model, urdf_load_path = load_model(mass_kg)
    q0 = _initial_configuration(model)
    problem_setup = build_problem(model, q0)
    solver = solve_reach(problem_setup, warm_xs, warm_us, verbose=verbose)
    print_solution_summary(solver, model, problem_setup["data"], problem_setup)

    probes = [build_wrench_probe(model, name) for name in FTS_FRAMES]
    sim_params = problem_setup["sim_params"]
    sim_params["fts_probes"] = [
        {
            "name": probe["name"],
            "hardware": probe["hardware"],
            "distal_mass": probe["distal_mass"],
            "origin_in_base": probe["placement"].translation.copy(),
        }
        for probe in probes
    ]
    sim_params["wrench_audit_max_error"] = None
    sim_params["wrench_max_magnitude"] = None

    if validate_wrench_flag:
        accelerations = solution_accelerations(solver.us, model.nv, len(solver.xs))
        max_error, max_magnitude = validate_wrench(
            model, probes, solver.xs, accelerations, mass_kg
        )
        sim_params["wrench_audit_max_error"] = max_error
        sim_params["wrench_max_magnitude"] = max_magnitude
        print(
            f"Wrench cross-check vs independent model: max error {max_error:.3e} "
            f"(peak wrench magnitude {max_magnitude:.3f})"
        )

    if with_plot:
        save_rss_reach_outputs(
            solver.xs,
            solver.us,
            model,
            problem_setup["data"],
            problem_setup["dt"],
            problem_setup["ee_frame_id"],
            CHEWIE_EE_POS_ENV,
            q0,
            urdf_load_path,
            out_dir,
            sim_params,
            probes,
        )

    if with_display or with_video:
        display_motion(
            solver,
            model,
            collision_model,
            visual_model,
            out_dir,
            problem_setup["dt"],
            with_display,
            with_video,
        )

    return solver, out_dir


def main(argv=None):
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    args = parse_args(argv)

    with_plot = "plot" in args.modes or "CROCODDYL_PLOT" in os.environ
    with_display = "display" in args.modes or "CROCODDYL_DISPLAY" in os.environ
    with_video = "video" in args.modes

    batch_stamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    is_batch = len(args.mass_kg) > 1 or args.batch_dir is not None
    if args.batch_dir:
        batch_root = os.path.abspath(args.batch_dir)
        os.makedirs(batch_root, exist_ok=True)
    elif is_batch:
        batch_root = os.path.join(SCRIPT_DIR, "..", "output", f"batch_{batch_stamp}")
        os.makedirs(batch_root, exist_ok=True)
    else:
        batch_root = None
    mass_dirs = []

    warm_xs = None
    warm_us = None
    last_solver = None

    for idx, mass_kg in enumerate(args.mass_kg):
        if args.out_dir and not is_batch:
            out_dir = args.out_dir
            os.makedirs(out_dir, exist_ok=True)
        elif batch_root is not None:
            out_dir = _make_output_dir(mass_kg, batch_root=batch_root)
        else:
            out_dir = _make_output_dir(mass_kg, batch_stamp)

        # Video runs for every mass (opens Meshcat briefly per mass); display loop is single-mass only.
        do_display = with_display and not is_batch
        do_video = with_video
        if with_display and is_batch and idx == 0:
            print(
                "Skipping interactive Meshcat loop for batch run "
                "(use video to save mp4 per mass, or single --mass-kg with display)."
            )

        solver, out_dir = run_single_mass(
            mass_kg,
            with_plot,
            do_display,
            do_video,
            out_dir,
            warm_xs=warm_xs,
            warm_us=warm_us,
            verbose=(idx == 0),
            validate_wrench_flag=args.validate_wrench,
        )
        warm_xs = solver.xs
        warm_us = solver.us
        last_solver = solver
        mass_dirs.append((mass_kg, out_dir))

    if batch_root is not None and with_plot:
        write_batch_readme(batch_root, mass_dirs)
        print(f"\nBatch outputs under: {os.path.abspath(batch_root)}")

    return last_solver


if __name__ == "__main__":
    main()
