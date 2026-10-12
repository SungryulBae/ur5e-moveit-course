# 6교시: MoveIt 2 + MuJoCo

## 오늘 할 것
- 로봇 모델(URDF)이 물리 시뮬레이터 MuJoCo의 모델로 어떻게 바뀌는지 본다
- ROS 2와 MuJoCo가 관절 데이터를 주고받는 구조(브리지)를 이해한다
- MoveIt → ROS 2 → MuJoCo로 로봇을 움직이고, 가상 로봇(mock)과의 차이를 확인한다

## 미리 알아 둘 것
| | 가상 로봇 (mock) | Gazebo | MuJoCo |
|---|---|---|---|
| 무게 · 중력 · 접촉 | 없음 (명령한 값 = 현재 값) | 있음 | 있음 |
| 속도 | 가장 빠름 | 보통 | 매우 빠름 |
| 주 용도 | 계획 · 로직 확인 | 센서까지 포함한 시뮬레이션 | **로봇 학습(AI)**, 제어 연구 |

- 로봇 AI는 시뮬레이터에서 수없이 시행착오를 하며 배웁니다 → 빠르고 정확한 물리 시뮬레이터가 필요합니다
- 오늘의 핵심: **같은 MoveIt 코드가 가상 로봇 → Gazebo → MuJoCo → 실제 UR5e에서 그대로 동작**합니다

---

## 터미널 배치
| 터미널 | 실행 | |
|---|---|---|
| 1 | `ros2 launch edu_arm_moveit_config demo.launch.py sim:=mujoco` | MoveIt + RViz + MuJoCo 창, 켜 둠 |
| 2 | 예제 코드를 차례로 | |

모든 터미널에서 먼저 `source ~/ur5e-moveit-course/scripts/activate.sh`
- MuJoCo를 직접 쓰는 스크립트는 `$MUJOCO_PYTHON`(설치 때 만든 MuJoCo 전용 파이썬)으로 실행합니다

## 실습 1. URDF → MuJoCo 모델
```bash
# 터미널 2
$MUJOCO_PYTHON -m day1_examples.c6_urdf_to_mujoco          # 변환 결과 출력 + 파일 저장
$MUJOCO_PYTHON -m day1_examples.c6_urdf_to_mujoco --view   # MuJoCo 창으로 보기 (창을 닫으면 종료)
```
✋ 출력에서 찾아보세요
- **액추에이터(모터)가 0개** → URDF에는 모터 정보가 없어서 직접 붙여야 합니다
- **등식 구속 1개** → 오른쪽 손가락이 왼쪽을 따라 움직이는 설정(mimic)이 자동으로 바뀐 것
- 관절 범위와 **토크 한계**(어깨 · 팔꿈치 150 Nm, 손목 28 Nm)가 그대로 넘어옴
- 현재 폴더에 생긴 `edu_arm_generated.xml`을 열어 `<actuator>`, `<equality>` 부분을 찾아보세요
- `--view`에서는 팔이 모든 관절 0° 자세(수평으로 쭉 뻗음)로 움직입니다 → 3교시 "관절이 모두 0이면 끝단은 어디?"의 답

## 실습 2. 브리지 구조 이해하기
```
 MoveIt
   │  "이 궤적대로 움직여라" (액션)           ▲ /joint_states (1초에 100번)
   ▼                                          │
 ┌──────────────── c6_mujoco_bridge ────────────────┐
 │ ① 궤적을 시간에 맞춰 나눠 → 모터 목표값          │
 │ ② MuJoCo 물리 계산 (0.002초씩, 실제 시간 속도)  │
 │ ③ 관절 각도 읽기 → /joint_states 로 보내기       │
 └──────────────────────────────────────────────────┘
```
- MoveIt은 상대가 가상 로봇인지 MuJoCo인지 모릅니다 → **액션 이름과 메시지 형식만 같으면** 됩니다
- 코드 `c6_mujoco_bridge.py`에서 `build_model()`, `apply_trajectory()`, `publish_joint_states()`를 찾아 읽어 보세요

## 실습 3. MoveIt → ROS 2 → MuJoCo
```bash
# 터미널 1
ros2 launch edu_arm_moveit_config demo.launch.py sim:=mujoco
# 터미널 2 — 4교시 코드를 그대로 실행
ros2 run day1_examples c4_pose_goal
ros2 run day1_examples c4_linear_gripper
ros2 run day1_examples c4_collision
```
- RViz와 MuJoCo 창의 로봇이 같이 움직입니다

## 실습 4. 가상 로봇 vs MuJoCo — 명령한 각도와 실제 각도
```bash
ros2 run day1_examples c6_gravity_check
```
- 같은 명령을 4교시 가상 로봇(`demo.launch.py`, sim 없이)에서도 실행해 비교해 보세요
- 가상 로봇은 오차 0°, MuJoCo는 팔을 뻗을수록 어깨가 더 처집니다(약 0.3° → 1.9°)

✋ 왜 처질까요? 실제 로봇 컨트롤러는 이 오차를 어떻게 없앨까요?

## 생각해 보기
1. MoveIt이 MuJoCo 로봇을 움직이려면 브리지가 어떤 이름의 액션을 제공해야 할까? 왜?
2. 가상 로봇에서는 문제없던 동작이 물리 시뮬레이터에서 실패한다면, 실제 로봇에 바로 올려도 될까?
3. URDF를 MuJoCo로 옮길 때 직접 추가해야 했던 것은?
