import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

NAMES = [f"joint_{i}" for i in range(1, 7)]


def main():
    rclpy.init()
    node = rclpy.create_node("jtc_probe")
    js = {}

    def cb(msg):
        for i, n in enumerate(msg.name):
            js[n] = msg.position[i]

    node.create_subscription(JointState, "/joint_states", cb, 10)
    client = ActionClient(node, FollowJointTrajectory,
                          "/joint_trajectory_controller/follow_joint_trajectory")
    while not client.wait_for_server(timeout_sec=10.0):
        pass
    node.get_logger().info("JTC connected")
    time.sleep(1.0)
    rclpy.spin_once(node, timeout_sec=1.0)
    start = [js.get(n, 0.0) for n in NAMES]
    node.get_logger().info(f"start: {[round(v, 3) for v in start]}")

    target = list(start)
    target[1] = 0.5
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = JointTrajectory(
        joint_names=NAMES,
        points=[JointTrajectoryPoint(
            positions=target,
            time_from_start=Duration(sec=3, nanosec=0))])
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    if not fut.result().accepted:
        node.get_logger().error("goal rejected")
        return
    node.get_logger().info("goal accepted, waiting 5s")
    time.sleep(5.0)
    rclpy.spin_once(node, timeout_sec=1.0)
    node.get_logger().info(f"joint_2 = {js.get('joint_2', '?')}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
