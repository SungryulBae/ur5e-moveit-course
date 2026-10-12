# 실습 환경 활성화
#
#   source scripts/activate.sh <내 번호>      예) source scripts/activate.sh 12
#
# - <내 번호> 는 ROS_DOMAIN_ID 가 된다 (0 ~ 101). 강의실에서는 강사가 정해 준 번호를 쓴다.
#   같은 번호를 쓰는 사람끼리는 로봇이 섞이므로, 모든 터미널에서 같은 번호로 활성화한다.
# - 번호를 생략하면 이전에 저장한 번호를 쓰고, 처음이면 30 을 쓴다.

_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_id_file="${_ws}/.ros_domain_id"

if [ -n "$1" ]; then
  echo "$1" > "${_id_file}"
fi
if [ -f "${_id_file}" ]; then
  export ROS_DOMAIN_ID="$(cat "${_id_file}")"
else
  export ROS_DOMAIN_ID=30
fi
# 같은 네트워크의 다른 PC 와 통신하지 않는다 (강의실에서 로봇이 섞이는 것 방지)
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

source /opt/ros/jazzy/setup.bash
if [ -f "${_ws}/install/local_setup.bash" ]; then
  source "${_ws}/install/local_setup.bash"
else
  echo "[주의] 아직 빌드 전입니다: cd ${_ws} && colcon build --symlink-install"
fi

# 실습 폴더 위치 (2일차 코드가 MuJoCo 로봇 모델을 찾을 때 사용)
export COURSE_WS="${_ws}"

# MuJoCo · PyTorch 가 설치된 가상환경의 파이썬 (1일차 6교시, 2일차)
if [ -x "${_ws}/.venv/bin/python" ]; then
  export MUJOCO_PYTHON="${_ws}/.venv/bin/python"
else
  unset MUJOCO_PYTHON
  echo "[주의] MuJoCo 가상환경이 없습니다 (\$MUJOCO_PYTHON 비어 있음): bash ${_ws}/setup/install.sh --skip-apt"
fi

echo "실습 환경 활성화: ${_ws} (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"
unset _ws _id_file
