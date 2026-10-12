"""5교시: 규칙 기반 전문가(Scripted Expert)와 규칙 기반 개입(Rule-Based Intervention)

HIL-SERL 에서는 사람이 스페이스마우스로 로봇을 붙잡아 고쳐 준다(Human Intervention).
여기서는 사람 대신 "규칙으로 짠 전문가"가 그 역할을 한다.
  - 시연(Demonstration): 학습 시작 전에 전문가가 몇 번 성공 시범을 보인다
  - 개입(Intervention) : 학습 중 에이전트가 헤매면 전문가가 잠깐 제어를 넘겨받는다

전문가는 상태(관찰)만 보고 행동을 정한다 → 에피소드 중간 어느 순간에 넘겨받아도 이어서 할 수 있다.
"""
import numpy as np

from day2_rl.envs import MAX_STEP

# 관찰 벡터에서 각 값의 위치 (envs.py 의 _obs 참고)
EE, GRIP, CUBE, GOAL = slice(0, 3), 3, slice(4, 7), slice(7, 10)


def _toward(ee, target, gain=1.0):
    """target 쪽으로 가는 행동 ([-1, 1], 한 스텝 최대 2 cm)."""
    return np.clip((target - ee) / MAX_STEP * gain, -1.0, 1.0)


def expert_reach(obs):
    return _toward(obs[EE], obs[GOAL])


def expert_push(obs):
    """큐브 뒤로 돌아가서 → 내려가서 → 목표 쪽으로 민다."""
    ee, cube, goal = obs[EE], obs[CUBE], obs[GOAL]
    d = goal[:2] - cube[:2]
    u = d / (np.linalg.norm(d) + 1e-6)              # 밀어야 하는 방향
    rel = ee[:2] - cube[:2]
    along = rel @ u                                  # 음수 = 큐브 뒤쪽
    lateral = np.linalg.norm(rel - along * u)        # 미는 선에서 옆으로 벗어난 거리
    push_z, safe_z = 0.025, 0.07
    behind = cube[:2] - u * 0.05

    # 밀고 있을 때 그리퍼 중심은 큐브 중심보다 약 2.5 cm 뒤에 있다 (큐브 반 변 2 cm + 손가락 두께)
    if along < -0.015 and lateral < 0.02 and ee[2] < 0.045:
        target = np.append(cube[:2] + u * 0.05, push_z)          # 큐브 뒤에 정렬됨 → 목표 쪽으로 민다
    elif np.linalg.norm(ee[:2] - behind) < 0.015:
        target = np.append(behind, push_z)                       # 뒤쪽 위에 도착 → 내려간다
    elif ee[2] < safe_z - 0.01 and np.linalg.norm(rel) < 0.09:
        target = np.append(ee[:2], safe_z)                       # 큐브 옆 · 앞에서 낮게 있으면 먼저 올라간다
    else:
        target = np.append(behind, safe_z)                       # 큐브 뒤쪽 위로 이동
    return _toward(ee, target)


def expert_pick(obs):
    """위로 가서 → 내려가서 → 잡고 → 목표로 들어 올린다. 4번째 값: + 닫기, - 열기."""
    ee, cube, goal, opening = obs[EE], obs[CUBE], obs[GOAL], obs[GRIP]
    dxy = np.linalg.norm(ee[:2] - cube[:2])
    near = np.linalg.norm(ee - cube) < 0.02
    holding = opening < 0.75 and near                # 손가락이 닫혀 있고 큐브가 손 안에 있음

    if holding:
        return np.append(_toward(ee, goal), 1.0)     # 잡은 채로 목표까지
    if near:
        return np.append(np.zeros(3), 1.0)           # 내려왔으면 닫기 (몇 스텝 걸림)
    if dxy < 0.012:
        return np.append(_toward(ee, cube + [0, 0, 0.0]), -1.0)       # 정확히 위에 있으면 내려가기
    above = cube + [0, 0, 0.07]
    if ee[2] < 0.05 and dxy > 0.03:
        return np.append(_toward(ee, np.append(ee[:2], 0.08)), -1.0)  # 낮은데 멀면 먼저 올라가기
    return np.append(_toward(ee, above), -1.0)       # 큐브 위로 이동 (손 벌린 채)


EXPERTS = {'reach': expert_reach, 'push': expert_push, 'pick': expert_pick}


class RuleBasedIntervention:
    """에이전트가 헤매면 전문가가 잠시 제어를 넘겨받는다.

    개입 조건 (사람이 "어, 저러면 안 되는데" 하고 손을 대는 순간을 규칙으로 흉내):
      - 최근 `patience` 스텝 동안 목표까지 거리가 `min_progress` 이상 줄지 않았다
    개입하면 `duration` 스텝 동안 전문가가 조종하고, 다시 에이전트에게 돌려준다.
    한 에피소드에 최대 `max_per_episode` 번까지만 개입한다 (전문가가 다 해 버리지 않도록).
    """

    def __init__(self, task, patience=8, min_progress=0.01, duration=10, max_per_episode=2):
        self.expert = EXPERTS[task]
        self.patience, self.min_progress = patience, min_progress
        self.duration, self.max_per_episode = duration, max_per_episode
        self.reset()

    def reset(self):
        self.history = []
        self.remaining = 0
        self.count = 0

    def __call__(self, obs, agent_action, distance):
        """실제로 실행할 행동과 개입 여부를 돌려준다."""
        self.history.append(distance)
        if self.remaining == 0 and self.count < self.max_per_episode and len(self.history) > self.patience:
            progress = self.history[-self.patience - 1] - self.history[-1]
            if progress < self.min_progress:
                self.remaining = self.duration
                self.count += 1
        if self.remaining > 0:
            self.remaining -= 1
            self.history = self.history[-1:]     # 개입 후에는 진척 기록을 새로 시작
            return self.expert(obs), True
        return agent_action, False
