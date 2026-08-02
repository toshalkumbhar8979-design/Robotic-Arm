"""Watch a live pick sequence: block poses, sim clock, finger joint.

Usage (inside container, after sourcing ROS):
  python3 /project/scripts/watch_pick.py
Triggers /arm_pick/trigger after 5 s and samples for ~45 s.
"""

import rclpy
from geometry_msgs.msg import PoseArray
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class Watcher(Node):

    def __init__(self):
        super().__init__("pick_watcher")
        self._blocks = {}
        self._sim = 0.0
        self._fingers = {}
        self.create_subscription(PoseArray, "/world/arm_world/pose/info",
                                 self._on_poses, 10)
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(JointState, "/joint_states",
                                 self._on_js, 10)
        self._client = self.create_client(Trigger, "/arm_pick/trigger")
        self._timing = self.create_timer(2.0, self._tick)
        self._count = 0

    def _on_poses(self, msg):
        for p in msg.poses:
            n = f"{p.position.x:.3f} {p.position.y:.3f} {p.position.z:.3f}"
            key = n
            if 0.15 < p.position.x < 0.35 and abs(p.position.y) < 0.2 \
                    and 0.7 < p.position.z < 1.0:
                self._blocks[key] = (p.position.x, p.position.y,
                                     p.position.z)

    def _on_clock(self, msg):
        self._sim = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_js(self, msg):
        for n, v in zip(msg.name, msg.position):
            if n == "left_finger_joint":
                self._fingers["lf"] = round(v, 2)
            if n == "right_finger_joint":
                self._fingers["rf"] = round(v, 2)

    def _tick(self):
        self._count += 1
        line = f"t={self._sim:6.1f}  blocks(on table)="
        table = [f"({b[0]:.3f},{b[1]:.3f},{b[2]:.3f})"
                 for b in self._blocks.values() if b[2] < 0.8]
        lifted = [f"({b[0]:.3f},{b[1]:.3f},{b[2]:.3f})"
                  for b in self._blocks.values() if b[2] >= 0.8]
        self.get_logger().info(
            f"{line}{table}  lifted/high={lifted}  fingers={self._fingers}")
        if self._count == 3:
            req = Trigger.Request()
            self._client.call_async(req)
            self.get_logger().info("triggered /arm_pick/trigger")
        if self._count >= 25:
            self.get_logger().info("watcher done")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = Watcher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
