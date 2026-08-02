import rclpy
from gz_msgs.msg import Pose_V


def main():
    rclpy.init()
    node = rclpy.create_node("pose_reader")
    data = {}

    def cb(msg):
        for p in msg.pose:
            pos = p.position
            data[p.name] = (round(pos.x, 3), round(pos.y, 3),
                            round(pos.z, 3))

    sub = node.create_subscription(Pose_V, "/world/arm_world/pose/info",
                                   cb, 10)
    rclpy.spin_once(node, timeout_sec=3.0)
    rclpy.spin_once(node, timeout_sec=3.0)
    for key in sorted(data):
        if "block" in key or "bucket" in key:
            node.get_logger().info(f"{key}: {data[key]}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
