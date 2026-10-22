"""Transform sim_simple.urdf into sim_rl.urdf for Isaac Lab RL use.

Changes applied:
  1. Converts both `floating` root joints to `fixed` with the home-position
     transforms from the collaborator's static_transform_publisher values.
  2. Strips all `<ros2_control>...</ros2_control>` blocks (5 total).
  3. Replaces every unresolvable `<mesh filename="package://{pkg}/...">` from
     rss / rss_ur20 / mikrolar_description / rad_bota_driver /
     zimmer_gripper_description with a 1 cm placeholder `<box>`. Keeps the
     `<visual>`/`<collision>` structure intact so the kinematic tree is
     unchanged -- Isaac Lab's UrdfConverter with `merge_fixed_joints=True`
     will fuse these small placeholders into their fixed-joint parents.

Run with:
    python3 build_sim_rl_urdf.py
"""

from __future__ import annotations

import math
import re
from pathlib import Path

SRC = Path(__file__).parent / "sim_simple.urdf"
DST = Path(__file__).parent / "sim_rl.urdf"

UNRESOLVED_PACKAGES = (
    "rss",
    "rss_ur20",
    "mikrolar_description",
    "rad_bota_driver",
    "zimmer_gripper_description",
)

# -------- step 1: floating -> fixed with home-position origins ------------

# The collaborator's quaternion (qx qy qz qw) = (0, 0, -0.260, 0.966) is a yaw
# rotation. Convert it to rpy for the URDF origin tag.
_qz, _qw = -0.260, 0.966
_yaw = 2.0 * math.atan2(_qz, _qw)  # ~ -0.5261 rad, i.e. -30 deg
ROTARY_ORIGIN = f'<origin rpy="0 0 {_yaw:.6f}" xyz="0 0 1.738"/>'
SLIDE_ORIGIN = '<origin rpy="0 0 0" xyz="-5.272 0.050 1.684"/>'

FLOATING_RE = re.compile(
    r'(<joint name="(?P<name>rotary_optical_plate_joint|slide_optical_plate_joint)") type="floating">\s*'
    r'(<parent link="[^"]+"/>)\s*'
    r'(<child link="[^"]+"/>)\s*'
    r'</joint>',
    re.DOTALL,
)


def _rewrite_floating(match: re.Match) -> str:
    name = match.group("name")
    origin = ROTARY_ORIGIN if name == "rotary_optical_plate_joint" else SLIDE_ORIGIN
    parent = match.group(3)
    child = match.group(4)
    return (
        f'{match.group(1)} type="fixed">\n'
        f'    {origin}\n'
        f'    {parent}\n'
        f'    {child}\n'
        f'  </joint>'
    )


# -------- step 2: strip ros2_control blocks -------------------------------

ROS2_CONTROL_RE = re.compile(
    r'[ \t]*<ros2_control\b.*?</ros2_control>\s*',
    re.DOTALL,
)

# -------- step 3: replace unresolved meshes with small placeholder boxes --

PKG_ALT = "|".join(re.escape(p) for p in UNRESOLVED_PACKAGES)
MESH_RE = re.compile(
    rf'<mesh filename="package://(?:{PKG_ALT})/[^"]+"\s*/>',
)
PLACEHOLDER = '<box size="0.01 0.01 0.01"/>'


# -------- driver ---------------------------------------------------------


def main() -> None:
    text = SRC.read_text()

    new_text, n_floating = FLOATING_RE.subn(_rewrite_floating, text)
    if n_floating != 2:
        raise SystemExit(f"expected 2 floating joints, rewrote {n_floating}")

    new_text, n_ros2 = ROS2_CONTROL_RE.subn("", new_text)
    if n_ros2 != 5:
        raise SystemExit(f"expected 5 ros2_control blocks, removed {n_ros2}")

    new_text, n_mesh = MESH_RE.subn(PLACEHOLDER, new_text)
    print(f"[info] replaced {n_mesh} unresolved mesh references")

    remaining_bad = [
        line for line in new_text.splitlines()
        if any(f"package://{p}/" in line for p in UNRESOLVED_PACKAGES)
    ]
    if remaining_bad:
        print("[warn] remaining references to unresolved packages:")
        for b in remaining_bad:
            print(f"   {b.strip()}")

    header = (
        "<!-- =================================================================================== -->\n"
        "<!-- |    sim_rl.urdf -- generated from sim_simple.urdf by build_sim_rl_urdf.py         | -->\n"
        "<!-- |    For Isaac Lab RL. Floating joints fixed, ros2_control blocks removed,          | -->\n"
        "<!-- |    unresolved meshes replaced with 1cm placeholder boxes.                         | -->\n"
        "<!-- =================================================================================== -->\n"
    )
    new_text = re.sub(r'\A<!--.*?-->\s*', header, new_text, count=1, flags=re.DOTALL)

    DST.write_text(new_text)
    print(f"[ok]  wrote {DST} ({len(new_text.splitlines())} lines)")


if __name__ == "__main__":
    main()
