"""MoveIt 2 SRDF generator for the BabyROS arm_6dof.

Planning semantics (see docs/ARCHITECTURE.md, planning ledger):
- arm: chain base_link -> tool0 (joints joint_1..joint_6)
- gripper: finger joints (both left_finger_joint and right_finger_joint active)
- arm_gripper: full chain base_link -> left_finger (arm + active finger joint)
- end effector: gripper_eef attached to tool0
- group states: home (all zeros), pre_pick (ready-to-reach posture),
  open (fingers neutral), closed (fingers gripping)
- virtual joint: fixed world -> base_link
- disabled collisions: adjacent link pairs only (parent/child), plus the
  left_finger/right_finger pair whose motion is mechanically coupled
"""

import xml.etree.ElementTree as ET

ARM_JOINTS = [
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6",
]
FINGER_JOINTS = ["left_finger_joint", "right_finger_joint"]

# URDF-native units: radians.
# Home is the FOLDED pose, NOT all zeros: the robot spawns with all
# joints at zero, where the gripper pads (which hang behind the palm)
# sit inside link_4's collision box. pick_node straightens the arm from
# the zero pose to this folded home with a raw joint trajectory before
# any planning; the SRDF home must match it so "go home" plans work.
HOME = {
    "joint_1": 0.0,
    "joint_2": 0.0,
    "joint_3": 0.0,
    "joint_4": 0.0,
    "joint_5": 1.5708,
    "joint_6": 0.0,
}
PRE_PICK = {
    "joint_1": 0.0,
    "joint_2": 1.2,
    "joint_3": -1.6,
    "joint_4": 0.0,
    "joint_5": 0.0,
    "joint_6": 0.0,
}

# Adjacent pairs from URDF topology (parent/child), finger pair coupled by
# and leaves with no neighbors beyond their parent. Cameras are mounted
# on the world link, which has no collision geometry.
DISABLED_PAIRS = [
    ("world", "base_link"),
    ("world", "camera_table_link"),
    ("world", "camera_stick_link"),
    ("base_link", "link_1"),
    ("link_1", "link_2"),
    ("link_2", "link_3"),
    ("link_3", "link_4"),
    ("link_4", "link_5"),
    ("link_5", "gripper_palm"),
    ("gripper_palm", "left_finger"),
    ("gripper_palm", "right_finger"),
    ("gripper_palm", "tool0"),
    ("gripper_palm", "camera_ee_link"),
    ("left_finger", "right_finger"),
    # The finger plates hang 100 mm below the palm, so in folded
    # poses they can sweep the upper arm/elbow area. The rest pose is
    # collision-free; these pairs are a planning safety net only.
    ("left_finger", "link_2"),
    ("right_finger", "link_2"),
    ("left_finger", "link_3"),
    ("right_finger", "link_3"),
    ("left_finger", "link_4"),
    ("right_finger", "link_4"),
    ("left_finger", "link_5"),
    ("right_finger", "link_5"),
]


def gen_srdf():
    robot = ET.Element("robot", {"name": "arm_6dof"})

    ET.SubElement(
        robot, "virtual_joint",
        {"name": "fixed_base", "type": "fixed",
         "parent_frame": "world", "child_link": "base_link"})

    arm = ET.SubElement(robot, "group", {"name": "arm"})
    ET.SubElement(arm, "chain", {"base_link": "base_link", "tip_link": "tool0"})

    gripper = ET.SubElement(robot, "group", {"name": "gripper"})
    for j in FINGER_JOINTS:
        ET.SubElement(gripper, "joint", {"name": j})

    arm_gripper = ET.SubElement(robot, "group", {"name": "arm_gripper"})
    ET.SubElement(arm_gripper, "chain", {"base_link": "base_link",
                                         "tip_link": "left_finger"})

    ET.SubElement(
        robot, "end_effector",
        {"name": "gripper_eef", "parent_link": "gripper_palm",
         "group": "gripper", "parent_group": "arm"})

    def state(name, group, values):
        s = ET.SubElement(robot, "group_state",
                          {"name": name, "group": group})
        for j, v in values.items():
            ET.SubElement(s, "joint", {"name": j, "value": f"{v:g}"})

    state("home", "arm", HOME)
    state("pre_pick", "arm", PRE_PICK)
    state("open", "gripper", {"left_finger_joint": 0.0,
                              "right_finger_joint": 0.0})
    state("closed", "gripper", {"left_finger_joint": 0.030,
                                "right_finger_joint": 0.030})

    for l1, l2 in DISABLED_PAIRS:
        ET.SubElement(
            robot, "disable_collisions",
            {"link1": l1, "link2": l2,
             "reason": "Adjacent or mimic-coupled"})

    return {"xml": robot, "urdf": "../../arm_description/urdf/arm_6dof.urdf"}
