# 4교시: MoveIt 2 모션 플래닝

## 오늘 할 것
- MoveIt 2가 무엇을 대신 해 주는지 이해한다 (IK · 경로 계획 · 충돌 검사 · 궤적 생성)
- "목표 자세 → IK → 경로 계획 → 실행"을 코드로 해 본다
- 직선 운동과 그리퍼 열기 · 닫기, 장애물 피하기를 해 본다

## 미리 알아 둘 것
```
 목표 자세 ─▶ move_group (IK · 경로 계획 · 충돌 검사 · 궤적) ─▶ 컨트롤러 ─▶ 하드웨어
                    ▲                                                    │
                    └──────────────── /joint_states ◀────────────────────┘
```
- **URDF**: 로봇의 몸 (2교시)
- **SRDF**: MoveIt용 추가 정보 — 플래닝 그룹(`arm`, `gripper`), 이름 붙인 자세(`ready`, `home`, `open`, `close`)
- 3교시에 직접 짠 FK · IK를 MoveIt이 대신 하고, **충돌 없는 경로**와 **속도가 붙은 궤적**까지 만들어 줍니다

---

## 터미널 배치
| 터미널 | 실행 | |
|---|---|---|
| 1 | `ros2 launch edu_arm_moveit_config demo.launch.py` | 로봇 + MoveIt + RViz, 켜 둠 |
| 2 | 예제 코드를 차례로 | 각각 끝나면 스스로 종료 |

시작 전: 3교시 터미널이 모두 꺼져 있는지 `ros2 node list`로 확인

## 실습 1. RViz에서 마우스로 계획하고 실행하기
```bash
# 터미널 1
source ~/ur5e-moveit-course/scripts/activate.sh
ros2 launch edu_arm_moveit_config demo.launch.py
```
- 왼쪽 MotionPlanning 패널 → Planning Group: `arm`
- 그리퍼 끝의 화살표 · 고리(인터랙티브 마커)를 끌어 목표 지정 → **Plan** → **Execute**
- Goal State에서 `ready`, `home`을 골라 보기

(선택) Gazebo 물리 시뮬레이터와 함께 띄우기: `ros2 launch edu_arm_moveit_config demo.launch.py sim:=gazebo`
→ RViz에서 계획하면 Gazebo 안의 로봇이 같이 움직입니다

## 실습 2. 코드로 목표 자세 주기
```bash
# 터미널 2
source ~/ur5e-moveit-course/scripts/activate.sh
ros2 run day1_examples c4_pose_goal
```
- 준비 자세 → 앞쪽 아래 → 왼쪽 → **팔이 닿지 않는 곳(일부러 실패)** → 준비 자세
- RViz에 끝단이 지나간 길이 **주황색 선**으로 그려집니다
- 로그의 `직선에서 최대 OO mm 벗어남`을 보세요 → 목표는 x, y, z로 줬지만 **가는 길은 관절 각도 공간에서 계획**되어 끝단이 휘어서 갑니다
- 코드: `moveit_helper.py`의 `move_to_pose()`

## 실습 3. 직선 운동 + 그리퍼
```bash
ros2 run day1_examples c4_linear_gripper
```
- 열기 → 집기 위치 위 → **직선 하강** → 닫기 → 직선 상승 → **옆으로 직선 이동** → 직선 하강 → 열기 → 직선 상승
- 직선 이동은 **초록색 선**: `직선에서 최대 0.0 mm 벗어남`
- `fraction` = 요청한 직선 중 실제로 계획된 비율. 100%가 아니면 실행하지 않습니다

| | 목표 자세 (`move_to_pose`) | 직선 이동 (`move_linear`) |
|---|---|---|
| 가는 길 | 관절 공간에서 자유롭게 → 끝단 곡선 | 끝단이 직선 |
| RViz 선 색 | 주황 | 초록 |
| 장점 | 장애물을 돌아갈 수 있음 | 경로를 예측할 수 있음 (물체 집기 · 용접) |
| 단점 | 끝단이 어디로 지나갈지 모름 | 장애물 · 특이점이 있으면 못 감 |

✋ 확인: 물체를 집으러 내려갈 때 그리퍼가 휘어서 들어가면 어떻게 될까요?

## 실습 4. 장애물 피하기
```bash
ros2 run day1_examples c4_collision
```
| 단계 | 로그 | RViz에서 볼 것 |
|---|---|---|
| 시작 | `충돌 물체 제거: wall (없음)` | 이전 실행에서 남은 벽 정리. 처음엔 "(없음)"이 정상 |
| 장애물 없이 A → B | 궤적 점 약 19개 | 주황 선이 거의 곧게 |
| 벽 추가 | `충돌 물체 추가: wall 완료` | A와 B 사이에 초록색 벽 |
| 벽이 있을 때 A → B | 궤적 점이 늘어남 | 주황 선이 **벽을 넘어가는** 모양 |
| 벽을 뚫는 직선 이동 | `fraction`이 매우 낮음 → 실행 안 함 | 로봇이 움직이지 않음 |

- MoveIt은 **알려 준 장애물만** 피합니다 (스스로 보지 못함)
- 계획은 무작위로 길을 찾기 때문에 실행할 때마다 경로 모양이 조금씩 다릅니다

## 생각해 보기
1. 3교시에 직접 짠 IK와 MoveIt이 하는 일의 차이는?
2. 벽을 MoveIt에 알려 주지 않고 실제 로봇을 움직이면?
3. 물체를 집을 때 직선 접근이 장애물 때문에 실패하면 어떻게 해야 할까?

**5교시는 다른 로봇(UR5e)을 씁니다. 모든 터미널을 Ctrl+C로 끄세요.**
