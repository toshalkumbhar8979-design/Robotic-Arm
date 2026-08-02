import itertools

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.srv import GetPositionIK

OQ = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)


def probe(client, node, x, y, z):
    req = GetPositionIK.Request()
    req.ik_request.group_name = "arm"
    req.ik_request.timeout = Duration(sec=0, nanosec=500_000_000)
    req.ik_request.avoid_collisions = True
    req.ik_request.pose_stamped.header.frame_id = "world"
    req.ik_request.pose_stamped.pose = Pose(
        position=__import__("geometry_msgs.msg",
                            fromlist=["Point"]).Point(x=x, y=y, z=z),
        orientation=OQ)
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    if fut.done() and fut.result() is not None:
        return fut.result().error_code.val == 1
    return False


def main():
    rclpy.init()
    node = rclpy.create_node("ik_probe4")
    client = node.create_client(GetPositionIK, "/compute_ik")
    while not client.wait_for_service(timeout_sec=10.0):
        pass

    for z in (0.85, 0.90, 0.95, 1.00):
        line = []
        for x, y in itertools.product((0.10, 0.15, 0.20, 0.25, 0.30),
                                      (-0.30, -0.20, -0.10, 0.0, 0.10, 0.20, 0.30)):
            line.append("X" if probe(client, node, x, y, z) else ".")
        node.get_logger().info(f"z={z:.2f}: " + " ".join(line))

    rclpy.shutdown()


if __name__ == "__main__":
    main()
