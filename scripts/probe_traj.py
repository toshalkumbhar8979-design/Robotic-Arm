import time

import rclpy
from moveit import MoveItPy

URDF = "/root/ros2_ws/install/arm_description/share/arm_description/urdf/arm_6dof.urdf"
SRDF = "/root/ros2_ws/install/arm_moveit_config/share/arm_moveit_config/srdf/arm_6dof.srdf"


def main():
    rclpy.init()
    node = rclpy.create_node("traj_probe")
    node.declare_parameter("robot_description", "")
    node.declare_parameter("robot_description_semantic", "")
    node.set_parameters([
        rclpy.parameter.Parameter("robot_description",
                                  value=open(URDF).read()),
        rclpy.parameter.Parameter("robot_description_semantic",
                                  value=open(SRDF).read()),
    ])
    rclpy.spin_once(node, timeout_sec=0.5)
    moveit = MoveItPy(node_name="traj_probe_mi")
    arm = moveit.get_planning_component("arm")
    node.get_logger().info("MoveItPy ready")

    arm.set_goal_state(configuration_name="home")
    plan_result = arm.plan()
    node.get_logger().info(
        f"plan home: error={plan_result.error_code.val}")

    traj = plan_result.trajectory.get_robot_trajectory_msg()
    jt = traj.joint_trajectory
    node.get_logger().info(f"joint names: {jt.joint_names}")
    times = [round(p.time_from_start.sec
                    + p.time_from_start.nanosec * 1e-9, 4)
             for p in jt.points]
    node.get_logger().info(f"n points: {len(jt.points)}")
    node.get_logger().info(f"times: {times}")
    if jt.points:
        node.get_logger().info(
            f"first positions: {jt.points[0].positions}")
        node.get_logger().info(
            f"last positions: {jt.points[-1].positions}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
