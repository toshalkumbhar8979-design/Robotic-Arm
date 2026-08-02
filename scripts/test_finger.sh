#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

watch() {
  timeout 1 ros2 topic echo /joint_states --once 2>/dev/null | tr -d '\0' | sed -n '/position:/,/velocity:/p' | tail -3 | tr -d ' \n'
  echo ""
}

echo "T0 left,right:"
watch
echo "== pub 0.8 (close) =="
ros2 topic pub --once /finger_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.8]}" >/dev/null 2>&1
for i in 1 2 3 4 5 6 7 8; do sleep 1; echo "t=${i}s"; watch; done
echo "== pub -0.8 (open) =="
ros2 topic pub --once /finger_controller/commands std_msgs/msg/Float64MultiArray "{data: [-0.8]}" >/dev/null 2>&1
for i in 1 2 3 4 5; do sleep 1; echo "t=${i}s"; watch; done
