#!/bin/bash
# Stage 3: run the full stack headless in the container
NAME=arm_ros
docker restart "$NAME" >/dev/null
sleep 2
docker exec "$NAME" bash -c '
for proc in move_group servo_node teleop_node teleop_buttons pick_node joy_node gz_sim_vendor parameter_bridge web_video_server serve_dashboard rosbridge_websocket gripper_action_server gz; do
  pkill -9 -x "$proc" 2>/dev/null && echo "leftover killed: $proc" || true
done
'
docker exec "$NAME" bash -c '
mkdir -p /tmp/.X11-unix
ln -sf /mnt/wslg/.X11-unix/X0 /tmp/.X11-unix/X0
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
nohup ros2 launch arm_bringup arm_full.launch.py use_gui:=true run_rviz:=false > /tmp/launch.log 2>&1 &
echo "launched pid $!"
'
sleep 50
docker exec "$NAME" bash -c 'echo "== launch.log tail =="; tail -20 /tmp/launch.log'
