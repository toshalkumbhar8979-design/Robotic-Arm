#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
timeout 15 ros2 service call /arm_pick/trigger std_srvs/srv/Trigger "{}"
