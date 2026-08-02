import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.srv import GetPositionIK

ORIENTS = {
    "x180": Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
    "old": Quaternion(x=0.0, y=-0.7071068, z=0.7071068, w=0.0),
    "xz90": Quaternion(x=0.7071068, y=0.0, z=0.0, w=0.7071068),
}


def probe(client, node, x, y, z, oq):
    req = GetPositionIK.Request()
    req.ik_request.group_name = "arm"
    req.ik_request.timeout = Duration(sec=0, nanosec=500_000_000)
    req.ik_request.avoid_collisions = True
    req.ik_request.pose_stamped.header.frame_id = "world"
    req.ik_request.pose_stamped.pose = Pose(
        position=__import__("geometry_msgs.msg",
                            fromlist=["Point"]).Point(x=x, y=y, z=z),
        orientation=oq)
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    if fut.done() and fut.result() is not None:
        return fut.result().error_code.val == 1
    return False


def main():
    rclpy.init()
    node = rclpy.create_node("ik_probe3")
    client = node.create_client(GetPositionIK, "/compute_ik")
    while not client.wait_for_service(timeout_sec=10.0):
        pass

    node.get_logger().info("== old-orientation drops at bucket spots ==")
    for x, y in ((0.28, 0.25), (0.32, 0.25), (0.32, -0.25), (0.25, 0.3)):
        ok = probe(client, node, x, y, 0.85, ORIENTS["old"])
        node.get_logger().info(f"old ({x},{y},0.85) -> {'IK' if ok else 'no'}")

    node.get_logger().info("== xz90 drops ==")
    for x, y in ((0.28, 0.25), (0.32, 0.25), (0.32, -0.25)):
        ok = probe(client, node, x, y, 0.85, ORIENTS["xz90"])
        node.get_logger().info(f"xz90 ({x},{y},0.85) -> {'IK' if ok else 'no'}")

    node.get_logger().info("== x-spine buckets, fingers-down ==")
    for x, y in ((0.25, 0.0), (0.30, 0.0), (0.35, 0.0), (0.40, 0.0)):
        ok = probe(client, node, x, y, 0.85, ORIENTS["x180"])
        node.get_logger().info(f"x180 ({x},{y},0.85) -> {'IK' if ok else 'no'}")

    node.get_logger().info("== x-spine old ==")
    for x, y in ((0.25, 0.0), (0.30, 0.0), (0.35, 0.0), (0.40, 0.0)):
        ok = probe(client, node, x, y, 0.85, ORIENTS["old"])
        node.get_logger().info(f"old ({x},{y},0.85) -> {'IK' if ok else 'no'}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
