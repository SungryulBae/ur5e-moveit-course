# 2일차: 강화학습과 HIL-SERL (MuJoCo UR5e + Robotiq 2F-85)

| 교시 | 주제 | 실습 안내 |
|---|---|---|
| 1 | 카메라 제어 및 데이터 수집 | [class1.md](class1.md) |
| 2 | 고전 강화학습 (MDP · 벨만 방정식 · Q-Learning) | [class2.md](class2.md) |
| 3 | Actor-Critic과 SAC | [class3.md](class3.md) |
| 4 | HIL-SERL과 RLPD 학습 구조 | [class4.md](class4.md) |
| 5 | MuJoCo HIL-SERL 환경 구성 | [class5.md](class5.md) |
| 6 | HIL-SERL 학습 실험 및 비교 | [class6.md](class6.md) |

## 시작하기
```bash
source ~/ur5e_course_ws/scripts/activate.sh
mkdir -p ~/day2_work && cd ~/day2_work     # 실습 결과(그림 · 데이터 · 학습 결과)가 여기에 저장됩니다
```
- 2일차 코드는 ROS 없이 파이썬으로 바로 실행합니다: `$MUJOCO_PYTHON -m day2_rl.<예제 이름>`
- `$MUJOCO_PYTHON`은 설치 때 만든 가상환경의 파이썬(MuJoCo · PyTorch 포함)입니다. 비어 있으면 `activate.sh`를 실행하지 않았거나 가상환경이 없는 것입니다 → `bash ~/ur5e_course_ws/setup/install.sh --skip-apt`
- 모든 학습은 **GPU 없이 CPU로** 몇 분 안에 끝나도록 작게 만들었습니다

## 로봇과 장면
- **UR5e** + **Robotiq 2F-85 그리퍼** (MuJoCo 공식 모델, 실물 로봇과 같은 구조)
- 바닥(작업대) 위의 빨간 큐브(한 변 4 cm), 초록 공 = 목표 위치
- 카메라 2대: `front`(작업대를 비스듬히 내려다봄), `wrist`(그리퍼 뒤에서 손가락 사이를 봄)

## 코드 구성 (`src/day2_rl/day2_rl/`)
| 파일 | 내용 | 교시 |
|---|---|---|
| `scene.py` | UR5e + 그리퍼 + 큐브 + 카메라 장면 만들기 | 1, 5 |
| `c1_camera_collect.py`, `c1_preprocess.py` | 카메라 데이터 수집 · 전처리 · 시각화 | 1 |
| `c2_gridworld.py` | 격자 세계: 가치 반복 · Q-Learning | 2 |
| `sac.py`, `c3_sac_pendulum.py` | SAC 직접 구현, 진자 세우기 학습 | 3 |
| `envs.py`, `experts.py`, `c5_env_check.py` | Reach · Push · Pick 환경, 규칙 기반 전문가 · 개입 | 5 |
| `train.py`, `c6_train.py`, `c6_compare.py` | RL Only / RLPD / HIL 학습과 비교 | 6 |
