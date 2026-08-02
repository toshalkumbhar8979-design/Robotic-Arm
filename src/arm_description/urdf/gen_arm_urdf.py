import math
import xml.etree.ElementTree as ET

ROBOT_NAME = "arm_6dof"

ARM_CONTROLLERS_YAML = "/root/ros2_ws/install/arm_description/share/arm_description/config/arm_controllers.yaml"

MATERIALS = {
    "silver": (0.75, 0.75, 0.75),
    "metallic_grey": (0.45, 0.45, 0.45),
    "servo_black": (0.12, 0.12, 0.12),
    "link_blue": (0.0, 0.35, 0.85),
    "deep_blue": (0.02, 0.2, 0.6),
    "connector_gold": (0.8, 0.5, 0.1),
}

FRICTION = {
    "base_link": 0.8,
    "link_1": 0.8,
    "link_2": 0.8,
    "link_3": 0.8,
    "link_4": 0.8,
    "link_5": 0.8,
    "gripper_palm": 1.0,
    "left_finger": 1.2,
    "right_finger": 1.2,
    "camera_ee_link": 0.5,
    "camera_table_link": 0.5,
    "camera_stick_link": 0.5,
}

# World-frame position of the robot base (table top height). The URDF root
# link is "world"; a fixed joint carries base_link to this height so that
# MoveIt, TF and Gazebo all agree on the base being 0.75 m above the floor.
BASE_WORLD_Z = 0.75

# Both cameras are rigidly attached to the "world" link, so the poses below
# ARE world coordinates (base sits on the table at z=0.75).
# Camera 1: on the table, looking across the table at the blocks.
TABLE_CAM_POS = (0.28, -0.33, 0.77)
TABLE_CAM_TARGET = (0.10, 0.0, 0.76)
# Camera 2: on a stick rising from the table, looking down at the blocks.
STICK_CAM_POS = (0.0, -0.42, 1.10)
STICK_CAM_TARGET = (0.10, 0.0, 0.76)
POLE_BASE = (0.0, -0.42, 0.75)

def camera_sensor(topic):
    # <topic> pins the gz-sim 8 scoped sensor topic (otherwise the sensor
    # publishes on the full entity path, which the bridge does not subscribe
    # to). CameraSensor appends "/camera_info" for the info topic.
    return f"""
<update_rate>30</update_rate>
<topic>{topic}</topic>
<camera>
  <horizontal_fov>1.134464</horizontal_fov>
  <image>
    <width>640</width>
    <height>480</height>
    <format>R8G8B8</format>
  </image>
  <clip>
    <near>0.05</near>
    <far>50</far>
  </clip>
</camera>
"""


def el(tag, **attrs):
    e = ET.Element(tag)
    for k, v in attrs.items():
        e.set(k, str(v))
    return e


def origin(xyz, rpy=None):
    o = el("origin", xyz=xyz)
    if rpy:
        o.set("rpy", rpy)
    return o


def geometry(kind, **attrs):
    g = el("geometry")
    g.append(el(kind, **attrs))
    return g


def material_block(name):
    r, g, b = MATERIALS[name]
    m = el("material", name=name)
    m.append(el("color", rgba=f"{r} {g} {b} 1.0"))
    return m


def visual(xyz, kind, material, attrs, rpy=None):
    v = el("visual")
    v.append(origin(xyz, rpy))
    v.append(geometry(kind, **attrs))
    v.append(material_block(material))
    return v


def collision(xyz, kind, attrs, rpy=None):
    c = el("collision")
    c.append(origin(xyz, rpy))
    c.append(geometry(kind, **attrs))
    return c


def inertial(mass, ixx, iyy, izz):
    i = el("inertial")
    i.append(el("mass", value=str(mass)))
    inertia = el("inertia", ixx=str(ixx), ixy="0", ixz="0",
                 iyy=str(iyy), iyz="0", izz=str(izz))
    i.append(inertia)
    return i


def link(name, visuals, collisions, inert, friction):
    l = el("link", name=name)
    for v in visuals:
        l.append(v)
    for c in collisions:
        l.append(c)
    if inert:
        l.append(inert)
    return l, friction


def joint(name, jtype, parent, child, xyz, axis=None,
          lower=None, upper=None, effort=None, velocity=None,
          damping=None, friction=None, mimic=None, rpy=None):
    j = el("joint", name=name, type=jtype)
    j.append(el("parent", link=parent))
    j.append(el("child", link=child))
    j.append(origin(xyz, rpy))
    if mimic:
        j.append(el("mimic", joint=mimic["joint"],
                    multiplier=mimic.get("multiplier", 1),
                    offset=mimic.get("offset", 0)))
    if axis:
        j.append(el("axis", xyz=axis))
    if lower is not None:
        limit = el("limit", lower=str(lower), upper=str(upper),
                   effort=str(effort), velocity=str(velocity))
        j.append(limit)
    dyn = []
    if damping is not None:
        dyn.append(f"damping=\"{damping}\"")
    if friction is not None:
        dyn.append(f"friction=\"{friction}\"")
    if dyn:
        j.append(ET.fromstring("<dynamics " + " ".join(dyn) + "/>"))
    return j


def vec_sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def vec_norm(v):
    return math.sqrt(sum(c * c for c in v))


def look_at_x(pos, target):
    d = vec_sub(target, pos)
    n = vec_norm(d)
    dx, dy, dz = d[0] / n, d[1] / n, d[2] / n
    return (0.0, -math.asin(dz), math.atan2(dy, dx))


def look_at_z(pos, target):
    d = vec_sub(target, pos)
    n = vec_norm(d)
    dx, dy, dz = d[0] / n, d[1] / n, d[2] / n
    return (0.0, math.acos(dz), math.atan2(dy, dx))


def fmt_rpy(rpy):
    return f"{rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}"


def gen_arm_links():
    links = []
    links.append(link(
        "base_link",
        [
            visual("0 0 0.004", "cylinder", "silver", {"radius": 0.09, "length": 0.008}),
            visual("0 0 0.04", "cylinder", "metallic_grey", {"radius": 0.065, "length": 0.07}),
        ],
        [collision("0 0 0.04", "cylinder", {"radius": 0.09, "length": 0.08})],
        inertial(1.5, 0.002, 0.002, 0.003),
        FRICTION["base_link"],
    ))
    links.append(link(
        "link_1",
        [
            visual("0 0 0.006", "cylinder", "link_blue", {"radius": 0.065, "length": 0.012}),
            visual("-0.045 0 0.045", "box", "link_blue", {"size": "0.02 0.065 0.08"}),
            visual("0.045 0 0.045", "box", "link_blue", {"size": "0.02 0.065 0.08"}),
            visual("0 0 0.04", "box", "servo_black", {"size": "0.035 0.035 0.055"}, rpy="0.5 0 0"),
        ],
        [collision("0 0 0.045", "cylinder", {"radius": 0.065, "length": 0.09})],
        inertial(1.0, 0.001, 0.001, 0.001),
        FRICTION["link_1"],
    ))
    links.append(link(
        "link_2",
        [
            visual("0 0.02 0.08", "box", "deep_blue", {"size": "0.038 0.038 0.16"}, rpy="0.15 0 0"),
            visual("0 0.005 0.04", "box", "deep_blue", {"size": "0.04 0.04 0.06"}, rpy="-0.1 0 0"),
            visual("0 0 0", "cylinder", "link_blue", {"radius": 0.025, "length": 0.05}, rpy="0 1.5708 0"),
            visual("0 0.035 0.155", "box", "servo_black", {"size": "0.032 0.05 0.032"}),
        ],
        [collision("0 0.02 0.08", "box", {"size": "0.04 0.04 0.16"}, rpy="0.15 0 0")],
        inertial(0.9, 0.002, 0.002, 0.0005),
        FRICTION["link_2"],
    ))
    links.append(link(
        "link_3",
        [
            visual("0 0.07 0", "box", "link_blue", {"size": "0.035 0.14 0.035"}),
            visual("0 0 0", "cylinder", "link_blue", {"radius": 0.022, "length": 0.045}, rpy="0 1.5708 0"),
            visual("0 0.04 0", "box", "servo_black", {"size": "0.03 0.045 0.03"}),
        ],
        [collision("0 0.07 0", "box", {"size": "0.035 0.14 0.035"})],
        inertial(0.6, 0.001, 0.0002, 0.001),
        FRICTION["link_3"],
    ))
    links.append(link(
        "link_4",
        [
            visual("0 0.0075 0", "cylinder", "link_blue", {"radius": 0.02, "length": 0.015}, rpy="1.5708 0 0"),
            visual("-0.016 0.025 0", "box", "link_blue", {"size": "0.012 0.035 0.045"}),
            visual("0.016 0.025 0", "box", "link_blue", {"size": "0.012 0.035 0.045"}),
            visual("0 0.025 0", "box", "servo_black", {"size": "0.024 0.03 0.024"}),
        ],
        [collision("0 0.02 0", "box", {"size": "0.04 0.04 0.04"})],
        inertial(0.3, 0.0001, 0.0001, 0.0001),
        FRICTION["link_4"],
    ))
    links.append(link(
        "link_5",
        [
            visual("0 0.015 0", "box", "link_blue", {"size": "0.025 0.03 0.025"}),
            visual("0 0.033 0", "cylinder", "silver", {"radius": 0.02, "length": 0.006}, rpy="1.5708 0 0"),
        ],
        [collision("0 0.02 0", "box", {"size": "0.03 0.04 0.03"})],
        inertial(0.2, 0.00005, 0.00005, 0.00005),
        FRICTION["link_5"],
    ))
    links.append(link(
        "gripper_palm",
        [
            visual("0 0.010 0", "box", "silver", {"size": "0.040 0.030 0.050"}),
            visual("0 -0.020 0", "cylinder", "metallic_grey",
                   {"radius": 0.022, "length": 0.032}, rpy="1.5708 0 0"),
        ],
        [collision("0 0.010 0", "box", {"size": "0.040 0.040 0.050"})],
        inertial(0.1, 0.00001, 0.00001, 0.00001),
        FRICTION["gripper_palm"],
    ))
    # Normal 2-finger parallel-jaw gripper (like a standard 6-DOF arm hand):
    # two rectangular fingers slide along ±X (left axis +X, right axis -X),
    # both closing toward the palm centre for the same commanded q. Joint
    # origins sit at the palm centre ±jaw_open, so inner faces rest at
    # |X| = jaw_open - q: q=0 gives an 80 mm gap (>> 20 mm cube + vision slop),
    # q=0.030 clamps a 20 mm cube. The fingers hang below the palm (palm -Y),
    # spanning the block height; a rubber pad sits on each inner (gripping)
    # face, thin in X, long in Y, wide in Z — same orientation as the finger.
    jaw_open = 0.0400
    FINGER_T = 0.010            # finger body thickness in X
    FINGER_L = 0.060            # finger body length in Y (down toward the table)
    FINGER_W = 0.030            # finger body width in Z
    PAD_T = 0.004               # rubber pad thickness in X (proud of inner face)
    PAD_L = 0.022               # rubber pad length in Y (spans the block height)
    PAD_W = 0.030               # rubber pad width in Z
    # Tool0 parks at block_z + 0.100 (0.86), so the palm sits at world z 0.815;
    # the block's table-top height is 0.75..0.77, i.e. palm_Y −0.045..−0.065.
    # The finger body therefore spans palm_Y −0.005..−0.065 (centre −0.035) and
    # the pad spans palm_Y −0.045..−0.065 (centre −0.055): the grip faces line
    # up with the block and the finger tips just clear the table top.
    FINGER_Y = -0.035
    PAD_Y = -0.056
    for side, sign in (("left", -1), ("right", 1)):   # left: -X side, +X motion
        # finger body local centre: inner face (toward the palm centre) at
        # X = 0, body extends AWAY from the centre by FINGER_T.
        bx = sign * FINGER_T / 2.0
        # rubber pad on the inner gripping face: extends from X = 0 toward the
        # centre by PAD_T (i.e. centre at -sign * PAD_T/2).
        px = -sign * PAD_T / 2.0
        links.append(link(
            f"{side}_finger",
            [
                visual(f"{bx} {FINGER_Y} 0", "box", "silver",
                       {"size": f"{FINGER_T} {FINGER_L} {FINGER_W}"}),
                visual(f"{px} {PAD_Y} 0", "box", "servo_black",
                       {"size": f"{PAD_T} {PAD_L} {PAD_W}"}),
            ],
            [
                collision(f"{bx} {FINGER_Y} 0", "box",
                          {"size": f"{FINGER_T} {FINGER_L} {FINGER_W}"}),
                collision(f"{px} {PAD_Y} 0", "box",
                          {"size": f"{PAD_T} {PAD_L} {PAD_W}"}),
            ],
            inertial(0.03, 0.000005, 0.000005, 0.000005),
            FRICTION[f"{side}_finger"],
        ))
    return links


def gen_sensor_links():
    links = []
    links.append(link(
        "tool0",
        [], [], None, 0.5,
    ))
    links.append(link(
        "camera_ee_link",
        [
            visual("0 0 0", "box", "servo_black", {"size": "0.012 0.01 0.012"}),
            visual("0.006 0 0", "cylinder", "silver", {"radius": 0.003, "length": 0.002}),
        ],
        [collision("0 0 0", "box", {"size": "0.012 0.01 0.012"})],
        inertial(0.01, 0.0000001, 0.0000001, 0.0000001),
        FRICTION["camera_ee_link"],
    ))
    # Table camera: small body sitting on the table, lens along +X (which the
    # gz camera renders down), aimed at the block cluster.
    table_rpy = fmt_rpy(look_at_x(TABLE_CAM_POS, TABLE_CAM_TARGET))
    links.append(link(
        "camera_table_link",
        [
            visual("0 0 0", "box", "metallic_grey", {"size": "0.05 0.03 0.03"}),
            visual("0.025 0 0", "cylinder", "servo_black",
                   {"radius": 0.008, "length": 0.012}, rpy="0 1.5708 0"),
        ],
        [collision("0 0 0", "box", {"size": "0.05 0.03 0.03"})],
        inertial(0.1, 0.0001, 0.0001, 0.0001),
        FRICTION["camera_table_link"],
    ))
    # Stick camera: pole from POLE_BASE up to STICK_CAM_POS, camera on top
    # aimed down at the block cluster. Pole geometry is expressed in the
    # stick-cam link frame (link origin == camera position).
    pole_rel = vec_sub(POLE_BASE, STICK_CAM_POS)
    pole_len = vec_norm(pole_rel)
    pole_mid = tuple((a + b) / 2.0 for a, b in zip(STICK_CAM_POS, POLE_BASE))
    pole_rpy = fmt_rpy(look_at_z(pole_mid, POLE_BASE))
    pole_origin = f"{pole_rel[0]:.4f} {pole_rel[1]:.4f} {pole_rel[2]:.4f}"
    stick_rpy = fmt_rpy(look_at_x(STICK_CAM_POS, STICK_CAM_TARGET))
    links.append(link(
        "camera_stick_link",
        [
            visual(pole_origin, "cylinder", "metallic_grey",
                   {"radius": 0.008, "length": f"{pole_len:.4f}"}, rpy=pole_rpy),
            visual("0 0 0", "box", "metallic_grey", {"size": "0.05 0.03 0.03"}),
            visual("0.025 0 0", "cylinder", "servo_black",
                   {"radius": 0.008, "length": 0.012}, rpy="0 1.5708 0"),
        ],
        [
            collision(pole_origin, "cylinder", {"radius": 0.012, "length": f"{pole_len:.4f}"}, rpy=pole_rpy),
            collision("0 0 0", "box", {"size": "0.05 0.03 0.03"}),
        ],
        inertial(0.1, 0.0001, 0.0001, 0.0001),
        FRICTION["camera_stick_link"],
    ))
    return links, table_rpy, stick_rpy


def gen_arm_joints():
    joints = []
    joints.append(joint(
        "joint_1", "revolute", "base_link", "link_1", "0 0 0.075",
        axis="0 0 1", lower=-6.2832, upper=6.2832, effort=50, velocity=2.0,
        damping=0.2, friction=0.1))
    joints.append(joint(
        "joint_2", "revolute", "link_1", "link_2", "0 0 0.065",
        axis="1 0 0", lower=-2.3562, upper=2.3562, effort=80, velocity=1.5,
        damping=0.2, friction=0.15))
    joints.append(joint(
        "joint_3", "revolute", "link_2", "link_3", "0 0.035 0.155",
        axis="1 0 0", lower=-2.3562, upper=2.3562, effort=50, velocity=2.0,
        damping=0.15, friction=0.08))
    joints.append(joint(
        "joint_4", "revolute", "link_3", "link_4", "0 0.14 0",
        axis="0 1 0", lower=-3.1416, upper=3.1416, effort=30, velocity=3.0,
        damping=0.08, friction=0.04))
    joints.append(joint(
        "joint_5", "revolute", "link_4", "link_5", "0 0.035 0",
        axis="0 0 1", lower=-2.3562, upper=2.3562, effort=20, velocity=4.0,
        damping=0.05, friction=0.03))
    joints.append(joint(
        "joint_6", "revolute", "link_5", "gripper_palm", "0 0.036 0",
        axis="0 1 0", lower=-3.1416, upper=3.1416, effort=10, velocity=5.0,
        damping=0.03, friction=0.02))
    # Symmetric prismatic finger jaws: both commanded (NO mimic - Gazebo
    # Dart ignores <mimic>, which broke the old right jaw). Left slides +X,
    # right slides -X for the SAME input q, both closing toward the palm
    # centre. |inner face| = jaw_open - q → open gap 80mm at q=0; a
    # 20mm cube is clamped at roughly q = jaw_open - 0.010 = 0.030.
    jaw_open = 0.0400
    joints.append(joint(
        "left_finger_joint", "prismatic", "gripper_palm", "left_finger",
        f"-{jaw_open:.4f} 0.0 0", axis="1 0 0",
        lower=0.0, upper=0.040, effort=50, velocity=0.05, damping=2.0,
        friction=0.2))
    joints.append(joint(
        "right_finger_joint", "prismatic", "gripper_palm", "right_finger",
        f"{jaw_open:.4f} 0.0 0", axis="-1 0 0",
        lower=0.0, upper=0.040, effort=50, velocity=0.05, damping=2.0,
        friction=0.2))
    joints.append(joint(
        "tool0_joint", "fixed", "gripper_palm", "tool0", "0 0.045 0", rpy="-1.5708 0 0"))
    joints.append(joint(
        "camera_ee_joint", "fixed", "gripper_palm", "camera_ee_link", "0 0.025 0.036", rpy="0 0 1.5708"))
    return joints


def add_ros2_control(root):
    block = el("ros2_control", name="GazeboSimSystem", type="system")
    hardware = el("hardware")
    plugin = el("plugin")
    plugin.text = "gz_ros2_control/GazeboSimSystem"
    hardware.append(plugin)
    block.append(hardware)
    controllable = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5",
                    "joint_6", "left_finger_joint", "right_finger_joint"]
    for name in controllable:
        j = el("joint", name=name)
        j.append(el("command_interface", name="position"))
        j.append(el("state_interface", name="position"))
        j.append(el("state_interface", name="velocity"))
        if name in ("left_finger_joint", "right_finger_joint"):
            for pname, pval in (("kp", "4000"), ("ki", "0"), ("kd", "80")):
                p = el("param", name=pname)
                p.text = pval
                j.append(p)
        block.append(j)
    root.append(block)


def friction_elements(mu):
    wrap = ET.fromstring(f'<wrap><mu1>{mu}</mu1><mu2>{mu}</mu2></wrap>')
    return list(wrap)


def add_gazebo_extensions(root):
    sensors = {
        "camera_table_link": "camera_table",
        "camera_stick_link": "camera_stick",
        "camera_ee_link": "camera_ee",
    }
    for ref, name in sensors.items():
        gz = el("gazebo", reference=ref)
        for child in friction_elements(FRICTION[ref]):
            gz.append(child)
        gz.append(ET.fromstring(
            '<sensor type="camera" name="' + name + '">'
            + camera_sensor("/" + name) + '</sensor>'))
        root.append(gz)
    for ref, mu in FRICTION.items():
        if ref in sensors:
            continue
        gz = el("gazebo", reference=ref)
        for child in friction_elements(mu):
            gz.append(child)
        root.append(gz)
    plugin = el("gazebo")
    plugin.append(ET.fromstring(
        '<plugin filename="libgz_ros2_control-system.so" '
        'name="gz_ros2_control::GazeboSimROS2ControlPlugin">'
        f'<parameters>{ARM_CONTROLLERS_YAML}</parameters>'
        '</plugin>'))
    root.append(plugin)
    sensors = el("gazebo")
    sensors.append(ET.fromstring(
        '<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>'))
    root.append(sensors)


def gen_urdf():
    root = el("robot", name=ROBOT_NAME)
    for name, rgba in MATERIALS.items():
        m = el("material", name=name)
        m.append(el("color", rgba=f"{rgba[0]} {rgba[1]} {rgba[2]} 1.0"))
        root.append(m)
    # Root link: world. A fixed joint places base_link on the table top so
    # MoveIt/TF and Gazebo share the same world frame (spawn at z=0).
    root.append(link(
        "world", [], [],
        inertial(0.001, 1e-6, 1e-6, 1e-6), 0.5)[0])
    root.append(joint(
        "world_joint", "fixed", "world", "base_link",
        f"0 0 {BASE_WORLD_Z}"))
    for l, _ in gen_arm_links():
        root.append(l)
    sensor_links, cam_rpy, stick_rpy = gen_sensor_links()
    for l, _ in sensor_links:
        root.append(l)
    for j in gen_arm_joints():
        root.append(j)
    root.append(joint(
        "camera_table_joint", "fixed", "world", "camera_table_link",
        f"{TABLE_CAM_POS[0]} {TABLE_CAM_POS[1]} {TABLE_CAM_POS[2]}", rpy=cam_rpy))
    root.append(joint(
        "camera_stick_joint", "fixed", "world", "camera_stick_link",
        f"{STICK_CAM_POS[0]} {STICK_CAM_POS[1]} {STICK_CAM_POS[2]}", rpy=stick_rpy))
    add_ros2_control(root)
    add_gazebo_extensions(root)
    return root


if __name__ == "__main__":
    ET.indent(gen_urdf())
    print(ET.tostring(gen_urdf(), encoding="unicode"))
