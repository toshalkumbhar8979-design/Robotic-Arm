"""GripperCommand action server for the BabyROS arm_6dof.

Implements control_msgs/action/GripperCommand on
/gripper_action_controller/gripper_cmd (the action used by
moveit_simple_controller_manager's GripperCommand controller entry), backed
by the forward_command_controller's position interface on
/finger_controller/commands.

The gripper has two symmetric prismatic jaws (left_finger_joint and
right_finger_joint), both independently commanded by the
forward_command_controller position interface on
/finger_controller/commands. It is NOT a mimic: the controller needs one
command per joint, so both values are published.
"""

import math
import time

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

FINGER_JOINTS = ["left_finger_joint", "right_finger_joint"]
FINGER_LIMIT = 0.040


class GripperActionServer(Node):

    def __init__(self):
        super().__init__("gripper_action_server")
        self.declare_parameter("action_name",
                               "/gripper_action_controller/gripper_cmd")
        self.declare_parameter("command_topic", "/finger_controller/commands")
        self.declare_parameter("position_tolerance", 0.02)
        self.declare_parameter("timeout", 5.0)

        self._tolerance = self.get_parameter("position_tolerance").value
        self._timeout = self.get_parameter("timeout").value

        self._finger_pos = 0.0
        self._got_state = False
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10)
        self._cmd_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter("command_topic").value, 10)

        self._server = ActionServer(
            self,
            GripperCommand,
            self.get_parameter("action_name").value,
            self._execute,
        )
        self.get_logger().info(
            f"GripperCommand action server on "
            f"{self.get_parameter('action_name').value}")

    def _on_joint_states(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name == FINGER_JOINTS[0]:
                self._finger_pos = pos
                self._got_state = True

    def _execute(self, goal_handle):
        goal = goal_handle.request.command
        target = max(-FINGER_LIMIT, min(FINGER_LIMIT, goal.position))
        self.get_logger().info(
            f"Gripper goal position={goal.position} -> clipped {target}")
        deadline = time.monotonic() + self._timeout

        cmd = Float64MultiArray()
        cmd.data = [target] * len(FINGER_JOINTS)
        self._cmd_pub.publish(cmd)

        while rclpy.ok():
            if self._got_state and abs(self._finger_pos - target) <= self._tolerance:
                goal_handle.succeed()
                result = GripperCommand.Result()
                result.position = self._finger_pos
                result.effort = 0.0
                result.stalled = False
                result.reached_goal = True
                self.get_logger().info("Gripper reached goal")
                return result
            if time.monotonic() > deadline:
                goal_handle.abort()
                result = GripperCommand.Result()
                result.position = self._finger_pos
                result.effort = 0.0
                result.stalled = True
                result.reached_goal = False
                self.get_logger().warn("Gripper goal timed out")
                return result
            feedback = GripperCommand.Feedback()
            feedback.position = self._finger_pos
            feedback.effort = 0.0
            feedback.stalled = False
            feedback.reached_goal = False
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)


def main():
    rclpy.init()
    rclpy.spin(GripperActionServer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
