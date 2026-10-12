"""6교시 실습 1: 학습 실행

  $MUJOCO_PYTHON -m day2_rl.c6_train --task push --mode rl     # RL Only
  $MUJOCO_PYTHON -m day2_rl.c6_train --task push --mode rlpd   # 시연 + RLPD
  $MUJOCO_PYTHON -m day2_rl.c6_train --task push --mode hil    # 시연 + RLPD + 규칙 기반 개입 (HIL-SERL)

결과: results/<task>_<mode>_s<seed>.json (학습 곡선), .pt (학습된 정책)
세 개를 서로 다른 터미널에서 동시에 돌린 뒤 c6_compare 로 비교한다.
"""
import argparse
import os

from day2_rl.train import train


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--task', default='push', choices=['reach', 'push', 'pick'])
    ap.add_argument('--mode', default='hil', choices=['rl', 'rlpd', 'hil'])
    ap.add_argument('--steps', type=int, default=20000)
    ap.add_argument('--reward', default='sparse', choices=['sparse', 'dense'])
    ap.add_argument('--demos', type=int, default=10)
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()
    os.makedirs('results', exist_ok=True)
    out = f'results/{args.task}_{args.mode}_s{args.seed}.json'
    train(args.task, args.mode, args.steps, args.reward, args.demos, args.seed,
          log=lambda m: print(m, flush=True), out=out)
    print(f'저장: {out}')


if __name__ == '__main__':
    main()
