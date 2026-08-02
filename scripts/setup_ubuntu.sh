#!/usr/bin/env bash
# Setup: Ubuntu (WSL 2) — install ROS 2 Lyrical + Gazebo Jetty + MoveIt stack
# for the BabyROS arm_6dof. Run inside WSL as the normal user.
set -euo pipefail

UBUNTU_CODENAME="$(lsb_release -cs)"
echo "== Ubuntu $UBUNTU_CODENAME =="

echo "== 1/5 System + ROS 2 repos =="
sudo apt update
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

echo "== 2/5 ROS 2 Lyrical (desktop + tooling) =="
sudo apt install -y \
  ros-lyrical-desktop \
  python3-rosdep python3-colcon-common-extensions \
  python3-argcomplete python3-vcstool git

echo "== 3/5 MoveIt 2 + controllers =="
sudo apt install -y \
  ros-lyrical-moveit \
  ros-lyrical-moveit-servo \
  ros-lyrical-moveit-ros-move-group \
  ros-lyrical-moveit-kinematics \
  ros-lyrical-moveit-planners-ompl \
  ros-lyrical-moveit-ros-visualization \
  ros-lyrical-moveit-commander \
  ros-lyrical-ros2-controllers \
  ros-lyrical-joy ros-lyrical-teleop-twist-joy

echo "== 4/5 Gazebo Jetty + ros_gz integration =="
sudo apt install -y \
  ros-lyrical-ros-gz \
  ros-lyrical-ros-gz-bridge \
  ros-lyrical-ros-gz-sim \
  ros-lyrical-gz-ros2-control \
  gz-sim10 \
  ros-lyrical-rosbridge-suite \
  ros-lyrical-web-video-server \
  ros-lyrical-camera-info-manager \
  python3-opencv python3-numpy

echo "== 5/5 Workspace + gamepad udev =="
mkdir -p ~/ros2_ws/src
sudo tee /etc/udev/rules.d/90-gamepad.rules > /dev/null <<'EOF'
# Give the user read access to USB gamepads (logitech-style)
KERNEL=="js*", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "== Env setup =="
grep -q "ros2/setup.bash" ~/.bashrc || cat >> ~/.bashrc <<'EOF'
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
EOF

source /opt/ros/lyrical/setup.bash
sudo rosdep init || true
rosdep update || true

echo "Done. Open a new shell (or: source ~/.bashrc)."
