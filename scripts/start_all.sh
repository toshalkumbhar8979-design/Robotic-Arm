#!/bin/bash
# ============================================================================
# start_all.sh - ONE command to bring up the entire BabyROS arm stack.
#
# Steps: (1) ensure the arm_ros container is running
#        (2) build all packages (stage2)
#        (3) kill any stale stack (stage3 cleanup)
#        (4) launch the full stack headless
#        (5) wait and print status + log tail
#
# Usage (from Windows PowerShell):
#   wsl -d kali-linux -- bash /mnt/c/Users/tosha/Downloads/Roboticarm/scripts/start_all.sh
# Or from WSL bash:
#   bash /mnt/c/Users/tosha/Downloads/Roboticarm/scripts/start_all.sh
# ============================================================================
set -u
NAME=arm_ros

echo "== [1/5] container $NAME =="
if ! docker ps --filter "name=$NAME" --format "{{.Names}}" | grep -q "^$NAME$"; then
  if docker ps -a --filter "name=$NAME" --format "{{.Names}}" | grep -q "^$NAME$"; then
    echo "container exists but stopped -> starting"
    docker start "$NAME"
  else
    echo "ERROR: container $NAME does not exist. Create it with scripts/run_container.sh first."
    exit 1
  fi
fi
docker exec "$NAME" bash -c 'echo "container up: $(hostname)"'

echo "== [2/5] build =="
docker exec "$NAME" bash -c '
set -e
source /opt/ros/jazzy/setup.bash
cd /root/ros2_ws
rm -rf /root/ros2_ws/src/*
cp -r /project/src/* /root/ros2_ws/src/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo > /tmp/build.log 2>&1
echo "BUILD EXIT=$?"
tail -4 /tmp/build.log
echo "== move python console scripts bin -> lib/<pkg> (launch/ros2 run require libexec) =="
for pkg in arm_teleop arm_pick web_dashboard; do
  if [ -d "install/$pkg/bin" ] && [ -n "$(ls install/$pkg/bin/)" ]; then
    mkdir -p "install/$pkg/lib/$pkg"
    mv install/$pkg/bin/* "install/$pkg/lib/$pkg/"
    rmdir install/$pkg/bin 2>/dev/null || true
    echo "$pkg -> $(ls install/$pkg/lib/$pkg/)"
  fi
done
'

echo "== [3/5] restart container (kills stale stack + reaps zombies) =="
docker restart "$NAME" >/dev/null
sleep 2
docker exec "$NAME" bash -c '
for proc in move_group servo_node teleop_node teleop_buttons pick_node joy_node gz_sim_vendor parameter_bridge web_video_server serve_dashboard rosbridge_websocket gripper_action_server gz; do
  pkill -9 -x "$proc" 2>/dev/null && echo "leftover killed: $proc" || true
done
ps -eo comm | grep -E "move_group|pick_node|gz_sim|serve_dash|teleop" | grep -v grep || echo "clean"
'

echo "== [4/5] launch =="
docker exec "$NAME" bash -c '
mkdir -p /tmp/.X11-unix
ln -sf /mnt/wslg/.X11-unix/X0 /tmp/.X11-unix/X0
ls -la /tmp/.X11-unix/
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
nohup ros2 launch arm_bringup arm_full.launch.py use_gui:=true run_rviz:=false > /tmp/launch.log 2>&1 &
echo "launched pid $!"
'

echo "== [5/5] waiting for stack, then status =="
sleep 60
docker exec "$NAME" bash -c '
echo "== launch.log tail =="
tail -20 /tmp/launch.log
echo ""
echo "== controllers =="
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
timeout 10 ros2 control list_controllers 2>/dev/null | tail -4
echo ""
echo "== pick sequence status =="
grep "arm_pick\]" /tmp/launch.log | tail -6
echo ""
echo "== URLs =="
echo "dashboard  : http://localhost:8000"
echo "camera     : http://localhost:8080/stream?topic=/camera_table/image_raw"
echo "rosbridge  : http://localhost:9090"
'
echo "DONE"
