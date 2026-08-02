from moveit.core import robot_trajectory
import inspect
for name in dir(robot_trajectory.RobotTrajectory):
    if not name.startswith("_"):
        print(name)