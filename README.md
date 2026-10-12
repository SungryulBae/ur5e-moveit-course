# UR5e로 배우는 ROS 2 로봇 제어 — 1일차 실습

ROS 2 Jazzy + MoveIt 2로 로봇팔을 움직여 보는 실습 자료입니다.
교육용 6축 로봇 **edu_arm**(UR5e와 같은 치수)으로 시작해서, 마지막에는 실제 **UR5e**와 물리 시뮬레이터 **MuJoCo**까지 같은 코드로 제어합니다.

## 준비물
- Ubuntu 24.04
- ROS 2 Jazzy ([설치 안내](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html))
- 인터넷 연결 (설치할 때만)

## 설치 (처음 한 번)
```bash
git clone https://github.com/SungryulBae/ur5e-moveit-course.git ~/ur5e-moveit-course
cd ~/ur5e-moveit-course
bash setup/install.sh          # ROS 패키지 설치(sudo) + MuJoCo + 빌드, 10~20분
```

## 터미널을 열 때마다
```bash
source ~/ur5e-moveit-course/scripts/activate.sh <내 번호>   # 예) ... activate.sh 12
```
- `<내 번호>`는 강사가 알려 주는 번호입니다. 한 번 입력하면 기억하므로, 다음부터는 번호 없이 `source .../activate.sh`만 해도 됩니다
- **모든 터미널에서** 실행해야 합니다. 안 하면 프로그램끼리 서로를 찾지 못합니다

## 실습 목차
| 교시 | 주제 | 실습 안내 |
|---|---|---|
| 2 | ROS 2와 로봇 관절 제어, 좌표계(TF) | [day1/class2.md](day1/class2.md) |
| 3 | DH 파라미터와 순기구학 · 역기구학 | [day1/class3.md](day1/class3.md) |
| 4 | MoveIt 2 모션 플래닝 | [day1/class4.md](day1/class4.md) |
| 5 | UR5e + ROS 2 연결 | [day1/class5.md](day1/class5.md) |
| 6 | MoveIt 2 + MuJoCo | [day1/class6.md](day1/class6.md) |

막히면: [day1/troubleshooting.md](day1/troubleshooting.md)

## 공통 규칙
1. **실습을 바꿀 때는 이전 터미널을 모두 Ctrl+C로 끄고** 시작합니다. 남아 있으면 로봇이 섞여서 이상하게 움직입니다
2. 확인: `ros2 node list` 결과가 비어 있으면 깨끗한 상태입니다

## 폴더 구성
```
ur5e-moveit-course/
├── day1/                      교시별 실습 안내
├── scripts/activate.sh        환경 활성화
├── setup/install.sh           설치
└── src/
    ├── edu_arm_description/   교육용 로봇 모델 (URDF), 관절 제어 실행 파일
    ├── edu_arm_moveit_config/ MoveIt 설정, 실행 파일 (mock / Gazebo / MuJoCo)
    └── day1_examples/         실습 코드 (c2_* ~ c6_*)
```
