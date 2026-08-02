import itertools

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Header

ORIENTS = {
    "x180": Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
    "y180": Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
}


def main():
    rclpy.init()
    node = rclpy.create_node("ik_probe2")
    client = node.create_client(GetPositionIK, "/compute_ik")
    while not client.wait_for_service(timeout_sec=10.0):
        pass
    node.get_logger().info("connected")

    for x, y in itertools.product((0.25, 0.28, 0.32), (0.25, 0.0, -0.25)):
        for oname, oq in ORIENTS.items():
            for avoid in (False, True):
                req = GetPositionIK.Request()
                req.ik_request.group_name = "arm"
                req.ik_request.timeout = Duration(sec=0, nanosec=500_000_000)
                req.ik_request.avoid_collisions = avoid
                req.ik_request.pose_stamped.header.frame_id = "world"
                req.ik_request.pose_stamped.pose = Pose(
                    position=__import__("geometry_msgs.msg",
                                        fromlist=["Point"]).Point(x=x, y=y,
                                                                  z=0.85),
                    orientation=oq)
                fut = client.call_async(req)
                rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
                ok = "??"
                if fut.done() and fut.result() is not None:
                    res = fut.result()
                    ok = "IK" if res.error_code.val == 1 else "no"
                node.get_logger().info(
                    f"({x:.2f},{y:+.2f},0.85) {oname} col={int(avoid)} -> {ok}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
