"""5교시 실습: 환경 확인 — 상태 · 행동 · 보상 · 성공 조건, 무작위 정책 vs 규칙 기반 전문가, 개입 동작

  $MUJOCO_PYTHON -m day2_rl.c5_env_check --task push           # 숫자로 확인
  $MUJOCO_PYTHON -m day2_rl.c5_env_check --task push --view    # MuJoCo 창으로 전문가 동작 보기
"""
import argparse
import time

import numpy as np

from day2_rl.envs import UR5eTaskEnv
from day2_rl.experts import EXPERTS, RuleBasedIntervention
from day2_rl.train import MAX_STEPS


def run(env, policy, episodes, seed=0):
    succ, rets = [], []
    for ep in range(episodes):
        obs, _ = env.reset(seed=seed + ep)
        done, ret = False, 0.0
        while not done:
            obs, r, term, trunc, info = env.step(policy(obs))
            ret += r
            done = term or trunc
        succ.append(info['success'])
        rets.append(ret)
    return np.mean(succ), np.mean(rets)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--task', default='push', choices=['reach', 'push', 'pick'])
    ap.add_argument('--view', action='store_true')
    args = ap.parse_args()
    task = args.task
    env = UR5eTaskEnv(task=task, reward_type='sparse', max_steps=MAX_STEPS[task])

    obs, _ = env.reset(seed=0)
    names = ['ee x', 'ee y', 'ee z', 'gripper open', 'cube x', 'cube y', 'cube z', 'goal x', 'goal y', 'goal z',
             'rel1 x', 'rel1 y', 'rel1 z', 'rel2 x', 'rel2 y', 'rel2 z']
    print(f'== 과제: {task}')
    print('상태(16):', ', '.join(f'{n}={v:+.3f}' for n, v in zip(names, obs)))
    print(f'행동: {env.action_space.shape[0]}차원 [-1, 1] (끝단 x, y, z 이동' + (', 그리퍼' if task == 'pick' else '') + ')')
    print(f'에피소드 최대 {env.max_steps} 스텝, 성공 조건: 거리 < {0.03 if task == "reach" else 0.04} m')

    rng = np.random.default_rng(0)
    random_policy = lambda o: rng.uniform(-1, 1, env.action_space.shape[0])   # noqa: E731
    for label, pol in [('무작위 정책', random_policy), ('규칙 기반 전문가', EXPERTS[task])]:
        sr, ret = run(env, pol, 20)
        print(f'{label:10s}: 성공률 {sr * 100:5.1f}%, 평균 보상 합 {ret:7.1f} (희소 보상: 실패한 스텝마다 -1)')

    # 개입 동작 보기: 무작위 정책 + 규칙 기반 개입
    helper = RuleBasedIntervention(task)
    obs, _ = env.reset(seed=1)
    helper.reset()
    log, done = [], False
    while not done:
        a, intervened = helper(obs, rng.uniform(-1, 1, env.action_space.shape[0]), env._distance())
        obs, r, term, trunc, info = env.step(a)
        log.append('E' if intervened else '.')
        done = term or trunc
    print('무작위 정책 + 개입 (E = 전문가가 조종한 스텝):', ''.join(log), '→', '성공' if info['success'] else '실패')

    if args.view:
        from mujoco import viewer as mj_viewer
        with mj_viewer.launch_passive(env.model, env.data) as v:
            for ep in range(5):
                obs, _ = env.reset(seed=ep)
                done = False
                while not done and v.is_running():
                    obs, r, term, trunc, info = env.step(EXPERTS[task](obs))
                    done = term or trunc
                    v.sync()
                    time.sleep(0.05)
                print(f'에피소드 {ep}: {"성공" if info["success"] else "실패"}')


if __name__ == '__main__':
    main()
