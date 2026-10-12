"""6교시 실습 2: RL Only vs RLPD vs HIL 학습 곡선 비교 + 학습된 정책 보기

  $MUJOCO_PYTHON -m day2_rl.c6_compare --task push                 # results/ 의 결과를 그래프로 (c6_compare.png)
  $MUJOCO_PYTHON -m day2_rl.c6_compare --task push --view hil      # 학습된 hil 정책을 MuJoCo 창으로 보기
"""
import argparse
import glob
import json
import time

import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

LABELS = {'rl': 'RL only (SAC)', 'rlpd': 'Demo + RLPD', 'hil': 'Demo + RLPD + Intervention (HIL)'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--task', default='push')
    ap.add_argument('--view', default=None, choices=['rl', 'rlpd', 'hil'])
    args = ap.parse_args()

    runs = {}
    for f in sorted(glob.glob(f'results/{args.task}_*_s*.json')):
        h = json.load(open(f))
        runs.setdefault(h['mode'], []).append(h)
    if not runs:
        print('results/ 에 결과가 없습니다. 먼저 c6_train 을 실행하세요.')
        return

    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    print(f'== {args.task}: 평가 성공률 (개입 없이, 10회)')
    for mode in ['rl', 'rlpd', 'hil']:
        if mode not in runs:
            continue
        steps = np.array(runs[mode][0]['step'])
        sr = np.array([r['eval_success'] for r in runs[mode]]) * 100
        axes[0].plot(steps, sr.mean(0), marker='o', ms=3, label=f'{LABELS[mode]} (n={len(sr)})')
        if len(sr) > 1:
            axes[0].fill_between(steps, sr.min(0), sr.max(0), alpha=0.15)
        first = next((int(s) for s, v in zip(steps, sr.mean(0)) if v >= 50), None)
        print(f'  {LABELS[mode]:34s} 마지막 {sr.mean(0)[-1]:5.1f}% | 처음 50% 넘은 스텝: {first or "없음"}')
        if mode == 'hil':
            ir = np.array([r['intervention_rate'] for r in runs[mode]]) * 100
            axes[1].plot(steps, ir.mean(0), color='tab:green', label='intervention rate (HIL)')
    axes[0].set_xlabel('environment steps')
    axes[0].set_ylabel('eval success rate (%)')
    axes[0].set_title(f'{args.task}: learning curves')
    axes[0].legend(fontsize=8)
    axes[1].set_xlabel('environment steps')
    axes[1].set_ylabel('% of steps controlled by expert')
    axes[1].set_title('How much help was needed')
    axes[1].legend(fontsize=8)
    plt.tight_layout()
    plt.savefig('c6_compare.png', dpi=110)
    print('그래프 저장: c6_compare.png')

    if args.view:
        from mujoco import viewer as mj_viewer

        from day2_rl.envs import UR5eTaskEnv
        from day2_rl.sac import SAC
        from day2_rl.train import MAX_STEPS
        env = UR5eTaskEnv(task=args.task, max_steps=MAX_STEPS[args.task])
        agent = SAC(env.observation_space.shape[0], env.action_space.shape[0], hidden=128,
                    layer_norm=args.view != 'rl')
        agent.load(f'results/{args.task}_{args.view}_s0.pt')
        with mj_viewer.launch_passive(env.model, env.data) as v:
            for ep in range(10):
                obs, _ = env.reset(seed=100 + ep)
                done = False
                while not done and v.is_running():
                    obs, r, term, trunc, info = env.step(agent.act(obs, deterministic=True))
                    done = term or trunc
                    v.sync()
                    time.sleep(0.05)
                print(f'에피소드 {ep}: {"성공" if info["success"] else "실패"}')


if __name__ == '__main__':
    main()
