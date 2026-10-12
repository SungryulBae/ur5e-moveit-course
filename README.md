# UR5e로 배우는 로봇 제어와 강화학습 — 실습 자료

- **1일차**: ROS 2 Jazzy + MoveIt 2로 로봇팔 제어. 교육용 6축 로봇 **edu_arm**(UR5e와 같은 치수)으로 시작해서 실제 **UR5e**와 물리 시뮬레이터 **MuJoCo**까지 같은 코드로 제어합니다
- **2일차**: MuJoCo **UR5e + Robotiq 2F-85**로 카메라 데이터 수집, 강화학습(Q-Learning · SAC), 시연 · 개입을 활용한 HIL-SERL 학습 구조를 직접 구현합니다 (GPU 없이 CPU로 실행)

## 준비물
- Ubuntu 24.04
- ROS 2 Jazzy ([설치 안내](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html))
- 인터넷 연결 (설치할 때만)

## 설치 (처음 한 번)
```bash
git clone https://github.com/SungryulBae/ur5e-moveit-course.git ~/ur5e_course_ws   # 폴더 이름: ur5e_course_ws
cd ~/ur5e_course_ws
bash setup/install.sh          # ROS 패키지(sudo) + MuJoCo · PyTorch + 로봇 모델 + 빌드, 10~20분
```

## 터미널을 열 때마다
```bash
source ~/ur5e_course_ws/scripts/activate.sh <내 번호>   # 예) ... activate.sh 12
```
- `<내 번호>`는 강사가 알려 주는 번호입니다. 한 번 입력하면 기억하므로, 다음부터는 번호 없이 `source .../activate.sh`만 해도 됩니다
- **모든 터미널에서** 실행해야 합니다. 안 하면 프로그램끼리 서로를 찾지 못합니다

## 1일차: 로봇 제어 기초 (ROS 2 · MoveIt 2)
| 교시 | 주제 | 실습 안내 |
|---|---|---|
| 2 | ROS 2와 로봇 관절 제어, 좌표계(TF) | [day1/class2.md](day1/class2.md) |
| 3 | DH 파라미터와 순기구학 · 역기구학 | [day1/class3.md](day1/class3.md) |
| 4 | MoveIt 2 모션 플래닝 | [day1/class4.md](day1/class4.md) |
| 5 | UR5e + ROS 2 연결 | [day1/class5.md](day1/class5.md) |
| 6 | MoveIt 2 + MuJoCo | [day1/class6.md](day1/class6.md) |

## 2일차: 강화학습과 HIL-SERL (MuJoCo) — [day2/README.md](day2/README.md)
| 교시 | 주제 | 실습 안내 |
|---|---|---|
| 1 | 카메라 제어 및 데이터 수집 | [day2/class1.md](day2/class1.md) |
| 2 | 고전 강화학습 (MDP · 벨만 방정식 · Q-Learning) | [day2/class2.md](day2/class2.md) |
| 3 | Actor-Critic과 SAC | [day2/class3.md](day2/class3.md) |
| 4 | HIL-SERL과 RLPD 학습 구조 | [day2/class4.md](day2/class4.md) |
| 5 | MuJoCo HIL-SERL 환경 구성 | [day2/class5.md](day2/class5.md) |
| 6 | HIL-SERL 학습 실험 및 비교 | [day2/class6.md](day2/class6.md) |

막히면: [day1/troubleshooting.md](day1/troubleshooting.md) (1 · 2일차 공통)

## 공통 규칙
1. **실습을 바꿀 때는 이전 터미널을 모두 Ctrl+C로 끄고** 시작합니다. 남아 있으면 로봇이 섞여서 이상하게 움직입니다
2. 확인: `ros2 node list` 결과가 비어 있으면 깨끗한 상태입니다

## 폴더 구성
```
ur5e_course_ws/
├── day1/                      1일차 교시별 실습 안내
├── day2/                      2일차 교시별 실습 안내
├── scripts/activate.sh        환경 활성화
├── setup/install.sh           설치
└── src/
    ├── edu_arm_description/   교육용 로봇 모델 (URDF), 관절 제어 실행 파일
    ├── edu_arm_moveit_config/ MoveIt 설정, 실행 파일 (mock / Gazebo / MuJoCo)
    ├── day1_examples/         1일차 실습 코드 (c2_* ~ c6_*)
    └── day2_rl/               2일차 실습 코드 (MuJoCo 장면 · 환경 · SAC · 전문가 · 학습)
```
