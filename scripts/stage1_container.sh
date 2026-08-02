#!/bin/bash
# Stage 1: Ubuntu 24.04 (noble) + ROS 2 Jazzy + MoveIt 2 + Gazebo Harmonic container
set -euo pipefail

NAME=arm_ros
IMG=ubuntu:24.04
PROJECT=/mnt/c/Users/tosha/Downloads/Roboticarm

echo "== pulling $IMG =="
docker pull "$IMG"

if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  docker rm -f "$NAME"
fi

echo "== starting container (WSLg GUI + published ports) =="
# NOTE: bridge network with published ports (NOT --network host): Docker
# Desktop does not forward host-network ports to the Windows browser, but
# published ports (-p) do. All stack services run inside this container, so
# bridge networking is transparent for them (gz transport is local).
# 8050 is deliberately NOT mapped here: the vision FastAPI dashboard backend
# runs on the Windows host on port 8050, and Docker's port proxy would block it.
docker run -d \
  --name "$NAME" \
  -p 5900:5900 \
  -p 6080:6080 \
  -p 8000:8000 \
  -p 8080:8080 \
  -p 9090:9090 \
  -v "$PROJECT":/project:ro \
  -v /mnt/wslg:/mnt/wslg \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}" \
  -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  "$IMG" sleep infinity

echo "== apt bootstrap (ROS repo + OSRF gazebo repo) =="
docker exec "$NAME" bash -c '
set -e
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y -qq curl gnupg lsb-release ca-certificates >/dev/null
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release; echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list
apt-get update -qq
' && echo "apt bootstrap OK"

echo "== installing ROS 2 Jazzy stack (this is the long step) =="
docker exec "$NAME" bash -c '
set -e
export DEBIAN_FRONTEND=noninteractive
PKGS="ros-jazzy-desktop
ros-jazzy-moveit
ros-jazzy-moveit-servo
ros-jazzy-joy
ros-jazzy-teleop-twist-joy
ros-jazzy-rosbridge-suite
ros-jazzy-ros-gz-sim
ros-jazzy-ros-gz-bridge
ros-jazzy-gz-ros2-control
ros-jazzy-controller-manager
ros-jazzy-ros2-controllers
xvfb
x11vnc
novnc
websockify
python3-colcon-common-extensions
python3-opencv
python3-numpy"
if ! apt-get install -y -qq $PKGS ros-jazzy-web-video-server > /tmp/install.log 2>&1; then
  echo "web-video-server missing, retrying without it"
  apt-get install -y -qq $PKGS > /tmp/install.log 2>&1 || { tail -50 /tmp/install.log; exit 1; }
fi
echo "INSTALL OK"
' && echo "=== STAGE 1 DONE ==="
