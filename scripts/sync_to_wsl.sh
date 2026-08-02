#!/usr/bin/env bash
# Sync the workspace from the Windows folder into WSL and build it.
# Run inside WSL (Ubuntu). Defaults target the standard checkout path.
set -euo pipefail

WIN_ROOT="${1:-/mnt/c/Users/tosha/Downloads/Roboticarm}"
WS="${2:-$HOME/ros2_ws}"

echo "== Syncing $WIN_ROOT/src -> $WS/src =="
mkdir -p "$WS/src"
rsync -a --delete --exclude '*.pyc' --exclude '__pycache__' \
  "$WIN_ROOT/src/" "$WS/src/"

echo "== Building =="
source /opt/ros/lyrical/setup.bash
cd "$WS"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source "$WS/install/setup.bash"

echo "== Packages =="
ros2 pkg list | grep -E '^arm_|^web_dashboard' || true
