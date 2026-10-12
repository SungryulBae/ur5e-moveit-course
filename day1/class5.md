# 5교시: UR5e + ROS 2 연결

## 오늘 할 것
- 실제 협동로봇(UR5e)이 ROS 2와 어떻게 연결되는지 이해한다
- ROS 2로 UR5e의 상태를 읽고, 움직이고, 입출력을 바꾼다
- 4교시 코드가 UR5e에서 거의 그대로 동작하는 것을 확인한다

> 실제 UR5e는 강사 PC 한 대에 연결해 시연합니다. 여러분은 **자기 PC의 가상 UR5e**로 같은 코드를 실행합니다.

## 미리 알아 둘 것
```
 [ROS 2 PC]                                          [UR5e 컨트롤러]
  MoveIt ─▶ scaled_joint_trajectory_controller
  ur_robot_driver ──── RTDE (로봇 상태) ◀─────────────  관절 · 입출력 · 안전 상태
       │       ◀─── External Control 프로그램 ───────  티칭펜던트에서 Play 하면
       │            (관절 명령)                          컨트롤러가 PC에 접속해 명령을 받음
       └────────── Dashboard (원격 버튼) ────────────▶  Play · Stop · 전원
```
- **UR 드라이버(`ur_robot_driver`)**가 ROS 2와 UR 컨트롤러 사이의 다리(브리지)입니다
- 실제 로봇은 **티칭펜던트에서 External Control 프로그램이 실행 중일 때만** ROS로 움직일 수 있습니다
- `scaled_joint_trajectory_controller`는 티칭펜던트의 **속도 슬라이더**를 반영합니다 (슬라이더 50% → 2배 느리게)
- 2교시 그림의 "하드웨어" 칸이 가상 로봇 → **UR 드라이버**로 바뀐 것뿐입니다

### 실제 로봇을 다룰 때의 안전 수칙
- 한 사람은 항상 **비상 정지 버튼 앞**에, 작업 반경(약 1 m) 안에는 아무도 들어가지 않는다
- 처음에는 속도 슬라이더 10~20%, **현재 위치에서 몇 cm씩만** 움직인다
- RViz에서 Plan으로 경로를 먼저 보고 이상이 없을 때 Execute

---

## 터미널 배치
| 터미널 | 실행 | |
|---|---|---|
| 1 | UR 드라이버 (가상 UR5e) | 켜 둠 |
| 2 | MoveIt | 켜 둠 |
| 3 | `c5_ur_status` | 켜 둠 (상태 계속 출력) |
| 4 | `c5_ur_move`, `c5_ur_io` | 차례로 |

모든 터미널에서 먼저 `source ~/ur5e-moveit-course/scripts/activate.sh`

## 실습 1. 가상 UR5e 띄우기
```bash
# 터미널 1
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy \
  use_mock_hardware:=true launch_rviz:=false initial_joint_controller:=scaled_joint_trajectory_controller
# 터미널 2 (10초쯤 기다린 뒤)
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```
- 실제 로봇이라면 `robot_ip:=로봇 IP`로 바꾸고 `use_mock_hardware:=true`를 뺍니다. 나머지는 모두 같습니다

## 실습 2. 상태 읽기
```bash
# 터미널 3 (켜 둠)
ros2 run day1_examples c5_ur_status
```
- 로봇 모드, 안전 모드, External Control 실행 여부, 속도 슬라이더, 디지털 출력, 끝단(TCP) 위치가 1초마다 출력됩니다
- 티칭펜던트 화면에 보이는 정보를 ROS 2로 받는 것입니다
- 처음 1초 정도는 값이 `?`로 나올 수 있습니다 (아직 받기 전). 다음 줄부터 값이 채워집니다

## 실습 3. MoveIt으로 UR5e 움직이기
```bash
# 터미널 4
ros2 run day1_examples c5_ur_move          # 안내가 나오면 Enter
```
- 준비 자세 → 위로 5 cm → 옆으로 5 cm → 옆 복귀 → 원위치 (모두 직선, 속도 10%)
- 4교시 코드(`moveit_helper.py`)를 그대로 쓰고 **로봇에 맞는 값만** 바꿨습니다

| | edu_arm (4교시) | UR5e |
|---|---|---|
| 플래닝 그룹 | `arm` | `ur_manipulator` |
| 궤적 컨트롤러 | `arm_controller` | `scaled_joint_trajectory_controller` |
| `base_link` 방향 | DH 기준과 같음 | DH 기준과 z축 180° 다름 → x, y 부호가 반대 |
| 목표 좌표 | 절대 좌표 | 현재 위치 기준 상대 이동 (실물 안전) |
| 속도 | 30% | 10% |

## 실습 4. ROS 2 명령을 UR5e로 전달하기
```bash
ros2 run day1_examples c5_ur_io            # 디지털 출력 0번 켜기 → 끄기, 속도 슬라이더 30% → 원래 값
```
- 터미널 3의 `디지털 출력`, `속도 슬라이더` 값이 바뀌는 것을 보세요
- 실제 로봇에서는 디지털 출력에 공압 그리퍼나 경광등이 연결되는 경우가 많습니다 (무엇이 연결돼 있는지 먼저 확인!)

## 생각해 보기
1. 티칭펜던트에서 External Control 프로그램을 멈추면 ROS에서 보낸 명령은 어떻게 될까?
2. 속도 슬라이더를 50%로 내리면 MoveIt이 계획한 3초짜리 궤적은 몇 초 걸릴까?
3. edu_arm 코드를 UR5e로 옮길 때 바꾼 것 세 가지는?

**6교시는 다시 edu_arm을 씁니다. 모든 터미널을 Ctrl+C로 끄세요.**
