# 6교시: HIL-SERL 학습 실험 및 비교

## 오늘 할 것
- 같은 과제를 세 가지 방식으로 학습하고 학습 곡선을 비교한다
- 개입 데이터가 학습을 얼마나 빠르게 하는지 숫자와 그래프로 확인한다

| 방식 | 시연 | RLPD (반반 섞기 · LayerNorm · UTD 2) | 개입 |
|---|---|---|---|
| `rl` (RL Only) | ✗ | ✗ | ✗ |
| `rlpd` | 성공 시범 10개 | ✓ | ✗ |
| `hil` (HIL-SERL) | 성공 시범 10개 | ✓ | ✓ 규칙 기반 |

---

## 실습 1. 먼저 짧게: Reach (희소 보상, 8,000 스텝, 각 약 2분)
```bash
# 터미널 1
cd ~/day2_work && $MUJOCO_PYTHON -m day2_rl.c6_train --task reach --mode rl --steps 8000
# 터미널 2 (동시에)
cd ~/day2_work && $MUJOCO_PYTHON -m day2_rl.c6_train --task reach --mode hil --steps 8000
# 둘 다 끝나면
$MUJOCO_PYTHON -m day2_rl.c6_compare --task reach
```
✋ hil의 **개입 비율**이 어떻게 변하는지 보세요. 에이전트가 배울수록 전문가가 나설 일이 줄어듭니다

참고: 강사 PC 결과 (실행마다 조금씩 다름)
| 방식 | 4,000 스텝 | 6,000 | 8,000 | 개입 비율 |
|---|---|---|---|---|
| RL Only | 0% | 0% | 0% | - |
| 시연 + RLPD + 개입 | 0% | 10% | **100%** | 46% → **0.3%** |

## 실습 2. 세 방식 동시에 학습 (Push, 희소 보상, 20,000 스텝)
터미널 3개에서 동시에 실행합니다 (각 CPU 약 3~5분)
```bash
# 터미널 1
cd ~/day2_work && $MUJOCO_PYTHON -m day2_rl.c6_train --task push --mode rl
# 터미널 2
cd ~/day2_work && $MUJOCO_PYTHON -m day2_rl.c6_train --task push --mode rlpd
# 터미널 3
cd ~/day2_work && $MUJOCO_PYTHON -m day2_rl.c6_train --task push --mode hil
```
1,000 스텝마다 출력되는 값
| 값 | 의미 |
|---|---|
| 평가 성공률 | **개입 없이**, 무작위성 없이 10번 시험한 성공률 (진짜 실력) |
| 학습 중 성공률 | 학습하면서 겪은 최근 20 에피소드 성공률 (hil은 개입 도움 포함) |
| 개입 비율 | 전체 스텝 중 전문가가 조종한 비율 |

✋ 학습이 도는 동안: hil의 "학습 중 성공률"은 높은데 "평가 성공률"은 0%인 구간이 있습니다. 왜일까요?

## 실습 3. 비교 · 학습된 정책 보기
```bash
$MUJOCO_PYTHON -m day2_rl.c6_compare --task push            # c6_compare.png
$MUJOCO_PYTHON -m day2_rl.c6_compare --task push --view hil  # 학습된 정책이 미는 모습
$MUJOCO_PYTHON -m day2_rl.c6_compare --task push --view rl   # 비교
```
- 왼쪽 그래프: 방식별 평가 성공률 / 오른쪽: hil의 개입 비율 변화
- 출력의 "처음 50% 넘은 스텝"으로 학습 속도 비교

참고: 강사 PC에서 한 번 실행한 결과 (실행마다 조금씩 달라집니다)
| 방식 | 10,000 스텝 | 20,000 스텝 |
|---|---|---|
| RL Only | 0% | 0% |
| 시연 + RLPD | 0% | 40% |
| 시연 + RLPD + 개입 | 0% | 60% |

## 실습 4 (시간이 남으면). 어려운 과제: Pick (희소 보상, 30,000 스텝, 각 약 7~8분)
```bash
cd ~/day2_work && $MUJOCO_PYTHON -m day2_rl.c6_train --task pick --mode rl   --steps 30000
cd ~/day2_work && $MUJOCO_PYTHON -m day2_rl.c6_train --task pick --mode rlpd --steps 30000
cd ~/day2_work && $MUJOCO_PYTHON -m day2_rl.c6_train --task pick --mode hil  --steps 30000
$MUJOCO_PYTHON -m day2_rl.c6_compare --task pick --view hil
```
참고: 강사 PC 결과
| 방식 | 15,000 | 20,000 | 25,000 | 30,000 |
|---|---|---|---|---|
| RL Only | 0% | 0% | 0% | 0% |
| 시연 + RLPD | 0% | 0% | 30% | 100% |
| 시연 + RLPD + 개입 | 10% | 20% | 80% | 100% |

✋ Pick에서는 hil의 **개입 비율이 끝까지 줄지 않습니다**(약 47%). 왜일까요?
→ 큐브를 잡으려고 그리퍼를 닫는 몇 스텝 동안은 목표까지 거리가 줄지 않아서, 개입 규칙이 "헤맨다"고 착각해 끼어듭니다.
   개입 규칙을 잘못 만들면 잘하고 있는데도 사람이 손을 대는 것과 같습니다 → `experts.py`의 `patience`를 늘려 보세요

## 더 해 보기
- 보상을 바꾸기: `--reward dense` → RL Only도 배울 수 있나요?
- 시연 개수: `--demos 2`, `--demos 30`
- 실행마다 결과가 다르므로 `--seed 1`, `--seed 2`로 여러 번 돌려 비교하면 더 정확합니다 (`c6_compare`가 평균을 그림)

## 생각해 보기
1. RL Only가 20,000 스텝 동안 거의 배우지 못한 이유는? (희소 보상 + 탐색)
2. 개입 데이터가 학습을 빠르게 한 이유를 4교시 그림으로 설명해 보세요
3. 실제 UR5e + 2F-85로 이 실험을 한다면, 규칙 기반 개입 대신 무엇이 필요할까? 안전은 어떻게 지킬까?
