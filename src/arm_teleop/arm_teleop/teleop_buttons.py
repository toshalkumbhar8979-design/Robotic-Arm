"""Button/axis teleop for the BabyROS arm_6dof.

Sends single-point JointTrajectory goals to the joint_trajectory_controller
for joint jogging, and direct position commands to the finger controller for
the gripper.

Supports multiple gamepad layouts via the ``controller_type`` parameter
(``auto`` | ``ps5`` | ``xbox`` | ``f710``):

  PS5 DualSense (hid-playstation)          Xbox One/Series (xpad)
  -------------------------------          ---------------------------
  Cross/Circle/Square/Triangle (0-3)       A/B/X/Y (0-3)
    -> joint_1, joint_2 jog                  -> joint_1, joint_2 jog
  L1(4)/R1(5) -> joint_3 jog               LB(4)/RB(5) -> joint_3 jog
  Share(8)/Options(9) -> gripper           Back(6)/Start(7) -> gripper
  D-Pad (12-15) -> joint_4, joint_5 jog    D-Pad (11-14) -> joint_4, joint_5
  L2/R2 (axes 4/5) -> analog gripper       LT/RT (axes 4/5) -> analog gripper

  F710 (XInput) keeps the original layout: buttons 0-5 jog joint_1..3,
  Back(6)/Start(7) -> gripper (no D-Pad in XInput mode).

Buttons are edge-triggered: hold-to-repeat is deliberately not implemented.
Triggers are continuous and mapped proportionally to the finger position.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6",
]
JOINT_RANGE = [-3.14, 3.14]

JOG_STEP = 0.15
FINGER_OPEN = 0.0
FINGER_CLOSE = 0.030

# Common jog buttons: A/X/Cross/Square (0-3) jog joint_1/2, LB/L1/RB/R1 (4-5)
# jog joint_3. Identical physical layout across F710, Xbox and PS5.
JOG_BUTTONS = {
    0: ("joint_1", -1), 1: ("joint_1", 1),
    2: ("joint_2", -1), 3: ("joint_2", 1),
    4: ("joint_3", -1), 5: ("joint_3", 1),
}

PROFILES = {
    "f710": {
        "gripper_close": 6,
        "gripper_open": 7,
        "dpad": None,  # F710 XInput exposes the D-Pad as a hat, not buttons
    },
    "xbox": {
        "gripper_close": 6,
        "gripper_open": 7,
        "dpad": {
            11: ("joint_4", -1), 12: ("joint_4", 1),
            13: ("joint_5", -1), 14: ("joint_5", 1),
        },
    },
    "ps5": {
        "gripper_close": 8,
        "gripper_open": 9,
        "dpad": {
            12: ("joint_4", -1), 13: ("joint_4", 1),
            14: ("joint_5", -1), 15: ("joint_5", 1),
        },
    },
}


def detect_profile(msg):
    """Guess the gamepad layout from the /joy message shape.

    Browser Gamepad API (rosbridge bridge) and Linux drivers report:
      PS5 DualSense: 6-12 axes, 18 buttons
      Xbox One/Series: 6 axes, 15-17 buttons
      Logitech F710 (XInput): 6 axes, 11 buttons
    """
    n_buttons = len(msg.buttons)
    if n_buttons >= 18:
        return "ps5"
    if n_buttons >= 12:
        return "xbox"
    return "f710"


class TeleopButtons(Node):

    def __init__(self):
        super().__init__("teleop_buttons")
        self.declare_parameter("jog_step", JOG_STEP)
        self.declare_parameter("controller_type", "auto")
        self.declare_parameter("trajectory_topic",
                               "/joint_trajectory_controller/"
                               "joint_trajectory")
        self.declare_parameter("finger_topic", "/finger_controller/commands")
        self._step = self.get_parameter("jog_step").value
        self._profile = self.get_parameter("controller_type").value

        self._traj_pub = self.create_publisher(
            JointTrajectory, self.get_parameter("trajectory_topic").value, 10)
        self._finger_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("finger_topic").value, 10)

        self._joint_positions = {j: 0.0 for j in ARM_JOINTS}
        self._finger_pos = FINGER_OPEN
        self._last_buttons = None
        self._detected = False

        self.create_subscription(
            Joy, "/joy", self._on_joy, 10)
        self.create_subscription(
            JointState, "/joint_states", self._on_js, 10)

    @property
    def map(self):
        if self._profile != "auto" and self._profile in PROFILES:
            return PROFILES[self._profile]
        return PROFILES["f710"]

    def _on_js(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self._joint_positions:
                self._joint_positions[name] = pos
            if name == "left_finger_joint":
                self._finger_pos = pos

    def _on_joy(self, msg):
        if not self._detected and self._profile == "auto":
            self._profile = detect_profile(msg)
            self.get_logger().info(
                f"Auto-detected gamepad layout: {self._profile} "
                f"({len(msg.axes)} axes, {len(msg.buttons)} buttons)")
            self._detected = True

        if self._last_buttons is None:
            self._last_buttons = list(msg.buttons)
            return
        pressed = [b and not p
                   for b, p in zip(msg.buttons, self._last_buttons)]
        self._last_buttons = list(msg.buttons)

        def jog(joint, direction):
            target = self._joint_positions[joint] + direction * self._step
            lo, hi = JOINT_RANGE
            target = max(lo, min(hi, target))
            self._send_trajectory({joint: target})

        for btn, (joint, sign) in JOG_BUTTONS.items():
            if btn < len(pressed) and pressed[btn]:
                jog(joint, sign)

        dpad = self.map["dpad"]
        if dpad:
            for btn, (joint, sign) in dpad.items():
                if btn < len(pressed) and pressed[btn]:
                    jog(joint, sign)

        close_btn = self.map["gripper_close"]
        open_btn = self.map["gripper_open"]
        if len(pressed) > close_btn and pressed[close_btn]:
            self._send_finger(FINGER_CLOSE)
        if len(pressed) > open_btn and pressed[open_btn]:
            self._send_finger(FINGER_OPEN)

        # Analog triggers -> proportional gripper (open on L2/LT, close on R2/RT)
        if len(msg.axes) > 4:
            lt = max(0.0, min(1.0, msg.axes[4]))
            rt = max(0.0, min(1.0, msg.axes[5]))
            target = self._finger_pos
            if rt > 0.05:
                target = FINGER_OPEN + (FINGER_CLOSE - FINGER_OPEN) * rt
            elif lt > 0.05:
                target = FINGER_OPEN + (FINGER_CLOSE - FINGER_OPEN) * (1.0 - lt)
            if abs(target - self._finger_pos) > 0.01:
                self._send_finger(target)

    def _send_trajectory(self, positions):
        msg = JointTrajectory()
        msg.joint_names = list(positions.keys())
        point = JointTrajectoryPoint()
        point.positions = list(positions.values())
        point.time_from_start.sec = 1
        msg.points.append(point)
        self._traj_pub.publish(msg)

    def _send_finger(self, position):
        msg = Float64MultiArray()
        # ForwardCommandController needs one value per joint (both jaws).
        msg.data = [position, position]
        self._finger_pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(TeleopButtons())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
