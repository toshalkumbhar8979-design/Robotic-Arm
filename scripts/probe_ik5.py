import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.srv import GetPositionIK

OQ = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

TARGETS = {
    "red_app": (0.22, 0.0, 0.895),
    "red_lift": (0.22, 0.0, 0.955),
    "green_app": (0.12, 0.15, 0.895),
    "green_lift": (0.12, 0.15, 0.955),
    "blue_app": (0.0, -0.18, 0.895),
    "blue_lift": (0.0, -0.18, 0.955),
    "drop_r": (0.30, 0.15, 0.85),
    "drop_g": (0.30, 0.0, 0.85),
    "drop_b": (0.30, -0.10, 0.85),
}


def probe(client, node, name, x, y, z):
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
    ok = "??"
    if fut.done() and fut.result() is not None:
        res = fut.result()
        ok = "IK" if res.error_code.val == 1 else f"e{res.error_code.val}"
    node.get_logger().info(f"{name:9s} ({x},{y},{z}) -> {ok}")


def main():
    rclpy.init()
    node = rclpy.create_node("ik_probe5")
    client = node.create_client(GetPositionIK, "/compute_ik")
    while not client.wait_for_service(timeout_sec=10.0):
        pass
    for name, (x, y, z) in TARGETS.items():
        probe(client, node, name, x, y, z)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
