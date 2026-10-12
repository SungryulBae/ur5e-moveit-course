"""6교시: 학습 실험 공통 코드 — RL Only / RLPD(시연) / HIL(시연 + 규칙 기반 개입)

  mode = 'rl'   : SAC 만. 처음 1,000 스텝은 무작위 행동으로 경험 수집
  mode = 'rlpd' : 학습 전에 전문가 시연 n_demos 개를 offline 버퍼에 넣고, 배치를 반반 섞어 학습 (RLPD)
  mode = 'hil'  : rlpd + 학습 중 에이전트가 헤매면 전문가가 개입, 개입 구간 데이터도 offline 버퍼에 추가 (HIL-SERL)
"""
import json
import time

import numpy as np

from day2_rl.envs import UR5eTaskEnv
from day2_rl.experts import EXPERTS, RuleBasedIntervention
from day2_rl.sac import SAC

MAX_STEPS = {'reach': 50, 'push': 60, 'pick': 60}


def collect_demos(env, task, n, seed=1000):
    """전문가 시연 n 개 (성공한 것만) → (o, a, r, o2, d) 목록."""
    data, ok, ep = [], 0, 0
    while ok < n:
        obs, _ = env.reset(seed=seed + ep)
        ep += 1
        traj, done = [], False
        while not done:
            a = EXPERTS[task](obs)
            obs2, r, term, trunc, info = env.step(a)
            traj.append((obs, a, r, obs2, float(term)))
            obs, done = obs2, term or trunc
        if info['success']:
            data += traj
            ok += 1
    return data


def evaluate(agent, env, n=10, seed=50_000):
    """개입 없이, 무작위성 없이(평균 행동) n 번 실행한 성공률."""
    succ = 0
    for i in range(n):
        obs, _ = env.reset(seed=seed + i)
        done = False
        while not done:
            obs, r, term, trunc, info = env.step(agent.act(obs, deterministic=True))
            done = term or trunc
        succ += info['success']
    return succ / n


def train(task='push', mode='hil', steps=20_000, reward_type='sparse', n_demos=10, seed=0,
          eval_every=1000, log=print, out=None):
    env = UR5eTaskEnv(task=task, reward_type=reward_type, max_steps=MAX_STEPS[task])
    eval_env = UR5eTaskEnv(task=task, reward_type=reward_type, max_steps=MAX_STEPS[task])
    obs_dim, act_dim = env.observation_space.shape[0], env.action_space.shape[0]
    rlpd = mode in ('rlpd', 'hil')
    agent = SAC(obs_dim, act_dim, hidden=128, layer_norm=rlpd, utd=2 if rlpd else 1, seed=seed)
    start = 1000 if mode == 'rl' else 200

    if rlpd:
        for t in collect_demos(env, task, n_demos):
            agent.offline.add(*t)
        log(f'시연 {n_demos}개 → offline 버퍼 {len(agent.offline)} 스텝')
    helper = RuleBasedIntervention(task) if mode == 'hil' else None

    hist = {'step': [], 'eval_success': [], 'train_success': [], 'intervention_rate': [],
            'task': task, 'mode': mode, 'seed': seed, 'reward_type': reward_type}
    obs, _ = env.reset(seed=seed)
    if helper:
        helper.reset()
    ep_succ, ep_interv, ep_steps, interv_steps, t0 = [], [], 0, 0, time.time()

    for step in range(1, steps + 1):
        a = agent.rng.uniform(-1, 1, act_dim) if step < start else agent.act(obs)
        intervened = False
        if helper:
            a, intervened = helper(obs, a, env._distance())
        obs2, r, term, trunc, info = env.step(a)
        t = (obs, a, r, obs2, float(term))
        agent.online.add(*t)
        if intervened:
            agent.offline.add(*t)          # 개입 데이터는 "좋은 예시"로 offline 버퍼에도
            interv_steps += 1
        ep_steps += 1
        obs = obs2
        if step >= start:
            agent.update()
        if term or trunc:
            ep_succ.append(info['success'])
            ep_interv.append(interv_steps / ep_steps)
            obs, _ = env.reset()
            if helper:
                helper.reset()
            ep_steps, interv_steps = 0, 0
        if step % eval_every == 0:
            sr = evaluate(agent, eval_env)
            hist['step'].append(step)
            hist['eval_success'].append(sr)
            hist['train_success'].append(float(np.mean(ep_succ[-20:])) if ep_succ else 0.0)
            hist['intervention_rate'].append(float(np.mean(ep_interv[-20:])) if ep_interv else 0.0)
            log(f'[{task}/{mode}] 스텝 {step:6d} | 평가 성공률 {sr * 100:5.1f}% | '
                f'학습 중 성공률 {hist["train_success"][-1] * 100:5.1f}% | '
                f'개입 비율 {hist["intervention_rate"][-1] * 100:4.1f}% | {time.time() - t0:.0f}s')
    if out:
        with open(out, 'w') as f:
            json.dump(hist, f)
        agent.save(out.replace('.json', '.pt'))
    return hist, agent
