"""Per-arm Pinocchio model rooted at the arm's F/T sensor, and the F/T wrench regressor.

Each UR20 sits on a 6-axis F/T sensor (han_fts / chewie_fts_combined). Everything above
the sensor (adapter, tombstone, UR base, 6 links, tool) is cut out of sim_chaser.urdf into
its own model whose root link IS the sensor frame, attached to the world by a free-flyer.
For that model the first 6 rows of inverse dynamics are exactly the wrench transmitted
through the sensor, expressed in the sensor frame:

    w_ft = rnea(q, v, a)[:6] = Y(q, v, a) @ pi

    q = [pose of the sensor frame in the world (7), arm joints J1..J6]
    v, a = [sensor-frame twist / spatial acceleration (6, local frame), joint rates (6)]
    pi = stacked Pinocchio dynamic parameters, 10 per body:
         [m, m*cx, m*cy, m*cz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]  (inertia about the body frame origin)

Body 0 (the root) is the rigid stack between the sensor and the UR base (tombstone etc.): it
only shows up through gravity and base acceleration. Fixed-joint links are merged into the
body they are attached to (camera, adapters, gripper -> wrist_3). The gripper fingers are
frozen (their prismatic joints move massless links).

Sign: w_ft is the wrench the mount applies to the arm ("mount_on_arm"). The sensor may
report the opposite ("arm_on_mount"); flip it before fitting.
"""
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

FT_LINK = {"han": "han_fts", "chewie": "chewie_fts_combined"}
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")
PARAM_NAMES = ("m", "mcx", "mcy", "mcz", "Ixx", "Ixy", "Iyy", "Ixz", "Iyz", "Izz")


def arm_urdf(urdf_path, arm):
    """URDF text of the subtree above the arm's F/T sensor, with the sensor link as root.
    Prismatic finger joints become fixed (massless fingers)."""
    root = ET.parse(urdf_path).getroot()
    joints = root.findall("joint")
    children = {}
    for j in joints:
        children.setdefault(j.find("parent").get("link"), []).append(j)
    keep_links, keep_joints, stack = {FT_LINK[arm]}, [], [FT_LINK[arm]]
    while stack:
        for j in children.get(stack.pop(), []):
            keep_joints.append(j)
            c = j.find("child").get("link")
            keep_links.add(c)
            stack.append(c)
    robot = ET.Element("robot", name=f"{arm}_ft")
    for link in root.findall("link"):
        if link.get("name") in keep_links:
            for tag in ("visual", "collision"):   # geometry is not needed for dynamics
                for e in link.findall(tag):
                    link.remove(e)
            robot.append(link)
    for j in keep_joints:
        if j.get("type") == "prismatic":
            j.set("type", "fixed")
            for tag in ("axis", "limit"):
                for e in j.findall(tag):
                    j.remove(e)
        robot.append(j)
    return ET.tostring(robot, encoding="unicode")


class ArmFT:
    """Pinocchio model of one arm above its F/T sensor + wrench regressor."""

    def __init__(self, urdf_path, arm):
        self.arm = arm
        self.model = pin.buildModelFromXML(arm_urdf(urdf_path, arm), pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        names = [self.model.names[i] for i in range(1, self.model.njoints)]
        assert names[1:] == [f"{arm}_{j}_joint" for j in JOINTS], names
        self.body_names = ["ft_stack (" + FT_LINK[arm] + " … base)"] + [
            self.model.names[i].replace(f"{arm}_", "").replace("_joint", "") for i in range(2, self.model.njoints)]
        self.nb = self.model.njoints - 1                 # 7 bodies
        self.pi_urdf = self.params_of(self.model)

    @staticmethod
    def params_of(model):
        return np.concatenate([model.inertias[i].toDynamicParameters() for i in range(1, model.njoints)])

    def set_params(self, pi):
        for k in range(self.nb):
            self.model.inertias[k + 1] = pin.Inertia.FromDynamicParameters(pi[10 * k:10 * k + 10])
        self.data = self.model.createData()

    def state(self, R_world_ft, qj, omega=None, dv=None, alpha=None, vj=None, aj=None):
        """Build (q, v, a). R_world_ft: sensor orientation (gravity = -z world).
        omega/alpha: sensor angular vel/acc, dv: linear acceleration of the sensor origin,
        all in the sensor frame. Missing terms are zero."""
        z3 = np.zeros(3)
        omega = z3 if omega is None else omega
        q = np.concatenate([z3, pin.Quaternion(R_world_ft).coeffs(), qj])
        v = np.concatenate([z3, omega, np.zeros(6) if vj is None else vj])
        # Pinocchio's free-flyer acceleration is spatial; convert from the classical
        # (point) acceleration of the sensor origin: a_spatial = a_point - omega x v_lin (v_lin=0 here)
        a = np.concatenate([z3 if dv is None else dv, z3 if alpha is None else alpha,
                            np.zeros(6) if aj is None else aj])
        return q, v, a

    def wrench(self, q, v, a):
        """Model-predicted sensor wrench (mount on arm) [Fx Fy Fz Tx Ty Tz], sensor frame."""
        return pin.rnea(self.model, self.data, q, v, a)[:6].copy()

    def regressor(self, q, v, a):
        """6 x (10*nb) regressor: wrench = Y @ pi."""
        return pin.computeJointTorqueRegressor(self.model, self.data, q, v, a)[:6].copy()

    def pseudo_inertia(self, p):
        """4x4 pseudo-inertia J(p) of one body (linear in p). J >= 0 <=> physically consistent."""
        m, h, (ixx, ixy, iyy, ixz, iyz, izz) = p[0], p[1:4], p[4:10]
        I = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        J = np.zeros((4, 4))
        J[:3, :3] = 0.5 * np.trace(I) * np.eye(3) - I
        J[:3, 3] = J[3, :3] = h
        J[3, 3] = m
        return J
