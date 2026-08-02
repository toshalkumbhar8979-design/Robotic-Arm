#!/bin/bash
NAME=arm_ros
docker exec "$NAME" bash -c '
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
timeout 30 ros2 launch arm_bringup arm_full.launch.py use_gui:=false run_rviz:=false --debug 2>&1 | grep -B2 -A25 "Traceback\|too many" | head -60
'
