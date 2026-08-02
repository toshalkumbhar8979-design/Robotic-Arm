#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
echo "=== joint_states ==="
timeout 5 ros2 topic echo /joint_states --once 2>/dev/null | tr -d '\0' | grep -A 2 "position:" | head -3
echo "=== block/bucket poses ==="
timeout 6 ros2 topic echo /world/arm_world/pose/info --once 2>/dev/null | tr -d '\0' | grep -A 1 -E 'name: "(block|bucket)'
