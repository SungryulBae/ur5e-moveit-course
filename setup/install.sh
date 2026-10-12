#!/usr/bin/env bash
# 실습 환경 설치 (Ubuntu 24.04 + ROS 2 Jazzy 가 이미 설치되어 있어야 한다)
#
#   bash setup/install.sh            # apt 패키지 + MuJoCo 가상환경 + 빌드
#   bash setup/install.sh --skip-apt # apt 설치는 건너뛰기 (이미 설치된 경우)
set -e

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "== 실습 폴더: ${WS}"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy 가 없습니다. 먼저 설치하세요: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"
  exit 1
fi

if [ "$1" != "--skip-apt" ]; then
  echo "== 1/3 ROS 패키지 설치 (sudo 비밀번호 필요)"
  sudo apt update
  sudo apt install -y \
    python3-colcon-common-extensions python3-venv python3-numpy \
    ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2 \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gripper-controllers \
    ros-jazzy-moveit ros-jazzy-moveit-planners-ompl ros-jazzy-pilz-industrial-motion-planner \
    ros-jazzy-moveit-simple-controller-manager ros-jazzy-moveit-ros-visualization \
    ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
    ros-jazzy-ur
else
  echo "== 1/3 apt 설치 건너뜀"
fi

echo "== 2/3 MuJoCo 가상환경 (${WS}/.venv)"
if [ ! -x "${WS}/.venv/bin/python" ]; then
  # --system-site-packages: 가상환경 안에서도 ROS 2 파이썬 패키지(rclpy 등)를 그대로 쓴다
  python3 -m venv --system-site-packages "${WS}/.venv"
fi
"${WS}/.venv/bin/pip" install --quiet --upgrade mujoco
"${WS}/.venv/bin/python" -c "import mujoco; print('   mujoco', mujoco.__version__)"

echo "== 3/3 빌드"
source /opt/ros/jazzy/setup.bash
cd "${WS}"
colcon build --symlink-install

echo
echo "설치 완료. 새 터미널마다 다음을 실행하세요 (번호는 강사가 정해 준 내 번호):"
echo "  source ${WS}/scripts/activate.sh <내 번호>"
