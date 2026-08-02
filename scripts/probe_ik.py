import itertools

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from std_msgs.msg import Header

BLOCKS = {
    "red": (0.22, 0.0),
    "green": (0.12, 0.15),
    "blue": (0.0, -0.18),
}

ORIENTS = {
    "old": Quaternion(x=0.0, y=-0.7071068, z=0.7071068, w=0.0),
    "ident": Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    "x180": Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
    "y180": Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
    "xz90": Quaternion(x=0.7071068, y=0.0, z=0.0, w=0.7071068),
    "x90n": Quaternion(x=-0.7071068, y=0.0, z=0.0, w=0.7071068),
}


def main():
    rclpy.init()
    node = rclpy.create_node("ik_probe")
    client = node.create_client(GetPositionIK, "/compute_ik")
    while not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().info("waiting for /compute_ik")
    node.get_logger().info("connected to /compute_ik")

    for (bname, (bx, by)), (oname, oq) in itertools.product(
            BLOCKS.items(), ORIENTS.items()):
        for z in (0.775,):
            req = GetPositionIK.Request()
            req.ik_request.group_name = "arm"
            req.ik_request.timeout = Duration(sec=0, nanosec=500_000_000)
            req.ik_request.avoid_collisions = True
            req.ik_request.pose_stamped.header.frame_id = "world"
            req.ik_request.pose_stamped.pose = Pose(
                position=__import__("geometry_msgs.msg",
                                    fromlist=["Point"]).Point(x=float(bx),
                                                              y=float(by),
                                                              z=z),
                orientation=oq)
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
            ok = "??"
            if fut.done() and fut.result() is not None:
                res = fut.result()
                ok = "IK" if res.error_code.val == 1 else f"e{res.error_code.val}"
            node.get_logger().info(
                f"{bname:5s} z={z:.2f} {oname:5s} -> {ok}")

    node.get_logger().info("== drop poses (x180) ==")
    for dname, (dx, dy) in {"red": (0.32, 0.25), "green": (0.32, 0.0),
                            "blue": (0.32, -0.25)}.items():
        for z in (0.90, 0.85, 0.80):
            req = GetPositionIK.Request()
            req.ik_request.group_name = "arm"
            req.ik_request.timeout = Duration(sec=0, nanosec=500_000_000)
            req.ik_request.avoid_collisions = True
            req.ik_request.pose_stamped.header.frame_id = "world"
            req.ik_request.pose_stamped.pose = Pose(
                position=__import__("geometry_msgs.msg",
                                    fromlist=["Point"]).Point(x=dx, y=dy, z=z),
                orientation=ORIENTS["x180"])
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
            ok = "??"
            if fut.done() and fut.result() is not None:
                res = fut.result()
                ok = "IK" if res.error_code.val == 1 else f"e{res.error_code.val}"
            node.get_logger().info(
                f"drop {dname:5s} z={z:.2f} -> {ok}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
