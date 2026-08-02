#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
echo "== who publishes finger commands (10s) =="
timeout 10 ros2 topic echo /finger_controller/commands 2>&1 | head -20
echo "== gripper_action_server alive? =="
ps -eo pid,comm | grep -iE 'gripper|pick' || echo "none"
