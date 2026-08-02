import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Quaternion, Point
from moveit_msgs.srv import GetPositionIK

OQ = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

PTS = [(0.3, -0.1, 1.0), (0.3, -0.1, 0.95), (0.3, -0.1, 0.90),
       (0.3, 0.0, 1.0), (0.3, 0.1, 1.0), (0.28, -0.1, 1.0)]


def main():
    rclpy.init()
    n = rclpy.create_node("p6")
    c = n.create_client(GetPositionIK, "/compute_ik")
    while not c.wait_for_service(timeout_sec=10.0):
        pass
    for (x, y, z) in PTS:
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.timeout = Duration(sec=0, nanosec=500_000_000)
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped.header.frame_id = "world"
        req.ik_request.pose_stamped.pose = Pose(
            position=Point(x=x, y=y, z=z), orientation=OQ)
        f = c.call_async(req)
        rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)
        ok = f.result().error_code.val if f.done() and f.result() else -1
        n.get_logger().info(f"({x},{y},{z}) -> {ok}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
