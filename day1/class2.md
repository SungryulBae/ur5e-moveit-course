# 2교시: ROS 2와 로봇 관절 제어, 좌표계(TF)

## 오늘 할 것
- ROS 2의 노드 · 토픽 · 서비스 · 액션이 로봇에서 어떻게 쓰이는지 본다
- 관절 각도와 좌표계(TF)의 관계를 눈으로 확인한다
- 코드로 로봇 관절을 직접 움직인다

## 미리 알아 둘 것: ROS 2 통신 4가지
| 방식 | 비유 | 로봇에서의 예 |
|---|---|---|
| 토픽 | 라디오 방송 (계속 흘러감) | `/joint_states` — 관절 각도를 1초에 100번 방송 |
| 서비스 | 전화 문의 (질문 1번 → 답 1번) | `/compute_ik` — "이 자세가 되려면 관절은 몇 도?" |
| 액션 | 택배 주문 (오래 걸림, 중간 상황 · 취소 · 결과) | `/arm_controller/follow_joint_trajectory` — "3초 동안 이 각도로 가라" |
| 파라미터 | 설정값 | `robot_description` — 로봇 모델(URDF) |

---

## 실습 1. 로봇 모델과 좌표계 보기
```bash
# 터미널 1
source ~/ur5e_course_ws/scripts/activate.sh
ros2 launch edu_arm_description display.launch.py
```
- RViz(3D 화면)와 슬라이더 창이 뜹니다
- 슬라이더를 하나씩 움직여 보세요. 로봇과 함께 좌표축(TF)이 움직입니다
- 좌표계 이름: `base_link`(바닥), `tool0`(손목 끝 플랜지), `tcp`(그리퍼 끝, 물체를 잡는 점)
- 슬라이더의 최소 · 최대값 = 관절이 돌 수 있는 범위 (UR5e와 같은 값: 대부분 ±360°, 팔꿈치만 ±180°)

✋ 확인: 슬라이더는 "진짜 로봇"을 움직인 걸까요? → 다음 실습에서 답을 찾습니다

**다음 실습 전에 터미널 1을 Ctrl+C로 끄세요**

## 실습 2. 진짜 제어 구조로 띄우기 (ros2_control)
```bash
# 터미널 1 (실습 4까지 켜 둠)
ros2 launch edu_arm_description control.launch.py
```
```
 [내 프로그램] ──"이 각도로 가라"(명령)──▶ [컨트롤러] ──▶ [하드웨어: 지금은 가상 로봇(mock)]
       ▲                                                        │
       └──────────── /joint_states (현재 각도, 상태) ◀──────────┘
```
- 슬라이더는 "지금 이 각도다"라는 **상태를 지어낸 것**이었고, 이제는 **명령을 보내면 하드웨어가 상태를 알려 주는** 진짜 구조입니다
- 가상 로봇(mock)을 실제 로봇이나 시뮬레이터로 바꿔도 위쪽 프로그램은 그대로 씁니다 (5 · 6교시)

터미널 2에서 하나씩 입력하고 결과를 보세요.
```bash
source ~/ur5e_course_ws/scripts/activate.sh
ros2 node list                          # 떠 있는 프로그램들 (슬라이더 노드가 없다!)
ros2 control list_controllers           # 컨트롤러 3개가 active
ros2 control list_hardware_interfaces   # [claimed] = 이 관절은 컨트롤러가 담당 중
ros2 action list                        # 관절 명령을 받는 액션
ros2 topic echo /joint_states --once    # 관절 7개의 현재 값
```
✋ 확인: `/joint_states`의 관절 순서가 이름순과 다르게 섞여 있나요? → 코드에서는 **순서가 아니라 이름으로** 찾아야 합니다

## 실습 3. 관절 각도 읽기 (토픽)
```bash
# 터미널 2 (켜 둠)
ros2 run day1_examples c2_joint_state_listener
```
- 관절 6개의 각도가 0.5초마다 도(°) 단위로 출력됩니다

## 실습 4. 관절 움직이기 (액션) + 끝단 위치 보기 (TF)
```bash
# 터미널 3
source ~/ur5e_course_ws/scripts/activate.sh
ros2 run day1_examples c2_joint_commander                       # 정해진 자세 4개를 차례로
ros2 run day1_examples c2_joint_commander 30 -90 90 -90 -90 0   # 원하는 각도(도)로
```
- RViz와 터미널 2의 숫자가 함께 바뀝니다
- 터미널 2를 Ctrl+C로 끄고 끝단 위치를 봅니다
```bash
# 터미널 2
ros2 run day1_examples c2_tf_listener      # base_link -> tool0 위치(m) · 자세(°)
```
- 터미널 3에서 다른 각도를 보내 보고, 끝단 위치가 어떻게 바뀌는지 보세요 → 3교시 순기구학으로 이어집니다

## 생각해 보기
1. `/joint_states`는 왜 서비스가 아니라 토픽일까?
2. "3초 동안 이 각도로 가라"는 왜 토픽이 아니라 액션일까?
3. 관절 각도로 끝단 위치를 계산해 준 프로그램은 무엇이었을까? (힌트: `ros2 node list`)

**3교시도 같은 `control.launch.py`를 씁니다. 터미널 1은 켜 두고, 터미널 2 · 3만 Ctrl+C로 끄세요.**
