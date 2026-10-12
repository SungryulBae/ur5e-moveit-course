"""3교시 실습: 직접 구현한 SAC 로 Pendulum(진자 세우기) 학습

  $MUJOCO_PYTHON -m day2_rl.c3_sac_pendulum               # 15,000 스텝 (CPU 약 1~2분)
  $MUJOCO_PYTHON -m day2_rl.c3_sac_pendulum --steps 5000  # 짧게

Pendulum: 아래로 늘어진 막대를 토크로 흔들어 세우고 버티는 문제
  상태: cosθ, sinθ, 각속도 / 행동: 토크 1개 / 보상: 똑바로 서 있을수록 0 에 가까움 (최대 0, 보통 -1500 ~ 0)
결과: 에피소드 점수 그래프 → sac_pendulum.png
"""
import argparse
import time

import gymnasium as gym
import matplotlib
import numpy as np

from day2_rl.sac import SAC

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', type=int, default=15000)
    ap.add_argument('--start', type=int, default=1000, help='처음 이만큼은 무작위 행동으로 경험을 모은다')
    args = ap.parse_args()

    env = gym.make('Pendulum-v1')
    act_scale = env.action_space.high[0]          # Pendulum 토크 범위 [-2, 2] ↔ SAC 행동 [-1, 1]
    agent = SAC(env.observation_space.shape[0], env.action_space.shape[0], hidden=128)
    obs, _ = env.reset(seed=0)
    ep_return, returns, t0 = 0.0, [], time.time()

    for step in range(1, args.steps + 1):
        a = agent.rng.uniform(-1, 1, 1) if step < args.start else agent.act(obs)
        obs2, r, term, trunc, _ = env.step(a * act_scale)
        agent.online.add(obs, a, r, obs2, float(term))
        obs, ep_return = obs2, ep_return + r
        if step >= args.start:
            info = agent.update()
        if term or trunc:
            returns.append(ep_return)
            if len(returns) % 10 == 0:
                msg = f'스텝 {step:6d} | 최근 10 에피소드 평균 점수 {np.mean(returns[-10:]):8.1f}'
                if step >= args.start:
                    msg += f' | α {info["alpha"]:.3f} | 엔트로피 {info["entropy"]:+.2f}'
                print(f'{msg} | {time.time() - t0:.0f}s', flush=True)
            obs, _ = env.reset()
            ep_return = 0.0

    plt.figure(figsize=(6, 3.5))
    plt.plot(returns, alpha=0.4, label='episode')
    k = 10
    plt.plot(np.arange(k - 1, len(returns)), np.convolve(returns, np.ones(k) / k, 'valid'), label='10-episode mean')
    plt.xlabel('episode')
    plt.ylabel('return')
    plt.title('SAC on Pendulum')
    plt.legend()
    plt.tight_layout()
    plt.savefig('sac_pendulum.png', dpi=120)
    print('그래프 저장: sac_pendulum.png')


if __name__ == '__main__':
    main()
