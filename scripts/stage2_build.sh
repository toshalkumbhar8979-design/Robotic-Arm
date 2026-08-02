#!/bin/bash
# Stage 2b: rebuild (after setup.py fixes) and launch
NAME=arm_ros
docker exec "$NAME" bash -c '
set -e
source /opt/ros/jazzy/setup.bash
cd /root/ros2_ws
rm -rf /root/ros2_ws/src/*
cp -r /project/src/* /root/ros2_ws/src/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo > /tmp/build3.log 2>&1
echo "BUILD EXIT=$?"
tail -6 /tmp/build3.log
echo "== move python console scripts bin -> lib/<pkg> (launch/ros2 run require libexec) =="
for pkg in arm_teleop arm_pick web_dashboard; do
  if [ -d "install/$pkg/bin" ] && [ -n "$(ls install/$pkg/bin/)" ]; then
    mkdir -p "install/$pkg/lib/$pkg"
    mv install/$pkg/bin/* "install/$pkg/lib/$pkg/"
    rmdir install/$pkg/bin 2>/dev/null || true
    echo "$pkg -> $(ls install/$pkg/lib/$pkg/)"
  fi
done
echo "== installed launch files =="
ls install/arm_pick/share/arm_pick/launch/ install/web_dashboard/share/web_dashboard/launch/ install/arm_teleop/share/arm_teleop/launch/ 2>&1
'
