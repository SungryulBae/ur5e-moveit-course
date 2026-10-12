# 막혔을 때

## 공통
| 증상 | 원인 · 해결 |
|---|---|
| `액션 서버가 없습니다`, `move_group 이 없습니다` | ① 터미널 1의 launch를 실행했는지 ② **그 터미널에서도** `activate.sh`를 실행했는지 (`echo $ROS_DOMAIN_ID`로 번호가 같은지 확인) |
| 로봇이 혼자 튀거나 이상하게 움직임, RViz 로봇 모델이 Error | 이전 실습의 프로그램이 남아 있음 → 모든 터미널 Ctrl+C → `ros2 node list`가 비었는지 확인 → 다시 실행 |
| `ros2 node list`에 이미 끈 노드가 보임 | 정보가 늦게 갱신된 것 → `ros2 daemon stop` 후 다시 확인 |
| `Package 'day1_examples' not found` | 빌드 안 됨 → `cd ~/ur5e-moveit-course && colcon build --symlink-install` 후 `activate.sh` 다시 실행 |
| 로그에 `Could not enable FIFO RT scheduling policy` | 무시해도 됩니다 (실시간 우선순위 권한이 없다는 안내) |

## 4교시 (MoveIt)
| 증상 | 원인 · 해결 |
|---|---|
| 계획 실패 (`FAILURE`, `NO_IK_SOLUTION`) | 목표가 팔이 닿지 않는 곳이거나 불가능한 자세인지 먼저 확인 |
| 직선 이동 `fraction`이 낮음 | 경로 중간에 장애물이 있거나, 팔이 다 펴지는 특이 자세를 지남 |
| RViz에 주황 · 초록 경로선이 안 보임 | 왼쪽 Displays 패널에서 **Day1 Markers** 체크 확인 |
| 그리퍼 모양이 로봇과 떨어져 혼자 떠 있음 | MoveIt의 계획용 시작 · 목표 표시 → MotionPlanning → Planning Request에서 Query Start/Goal State 끄기 |
| move_group 로그의 `No 3D sensor plugin(s) defined for octomap updates` | 무시해도 됩니다 (3D 카메라를 쓰지 않는 설정) |

## 5교시 (UR5e)
| 증상 | 원인 · 해결 |
|---|---|
| `c5_ur_status`에 `?`만 나옴 | UR 드라이버(터미널 1)가 아직 안 뜸 → 10초쯤 기다린 뒤 다시 |
| `c5_ur_move`에서 계획 실패 | MoveIt(터미널 2)을 띄웠는지 확인 |

## 6교시 (MuJoCo)
| 증상 | 원인 · 해결 |
|---|---|
| `No module named 'mujoco'` | `python3`가 아니라 `$MUJOCO_PYTHON`으로 실행. 값이 비어 있으면 `setup/install.sh`를 다시 실행 |
| MuJoCo 창이 안 뜸 | 화면이 없는 원격 환경 → `sim:=mujoco mj_viewer:=false`로 실행하고 RViz로 확인 |
| 명령이 두 번 실행되거나 로봇이 튐 | 브리지가 두 개 떠 있음 (이전 실행이 남음) → 모두 끄고 다시 |
