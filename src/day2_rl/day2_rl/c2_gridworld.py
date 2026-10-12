"""2교시 실습: 격자 세계(Grid World)에서 MDP · 벨만 방정식 · Q-Learning

  $MUJOCO_PYTHON -m day2_rl.c2_gridworld

격자 지도 (로봇이 작업대 위에서 물체까지 가는 상황을 단순화)
  S: 시작, G: 목표(+1, 끝), X: 장애물(-1, 끝), .: 빈칸(한 칸 움직일 때마다 -0.04)
  행동: 위 · 아래 · 왼쪽 · 오른쪽. 80% 확률로 원하는 방향, 10%씩 옆으로 미끄러짐 (확률적 전이)

1) 가치 반복(Value Iteration): 전이 확률을 "알 때" 벨만 방정식을 반복해 V 를 구한다
2) Q-Learning: 전이 확률을 "모를 때" 직접 움직여 보며 Q 를 배운다
→ 두 방법이 같은 정책에 도달하는지 비교. 그림: c2_values.png, c2_learning.png
"""
import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

GRID = [
    'S...',
    '.X.X',
    '...X',
    'X..G',
]
ACTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]   # 위, 아래, 왼쪽, 오른쪽
ARROWS = ['↑', '↓', '←', '→']
STEP_R, GOAL_R, PIT_R, GAMMA = -0.04, 1.0, -1.0, 0.95


class GridWorld:
    def __init__(self, slip=0.2):
        self.h, self.w = len(GRID), len(GRID[0])
        self.slip = slip
        self.start = next((r, c) for r in range(self.h) for c in range(self.w) if GRID[r][c] == 'S')

    def terminal(self, s):
        return GRID[s[0]][s[1]] in 'GX'

    def move(self, s, a):
        r, c = s[0] + ACTIONS[a][0], s[1] + ACTIONS[a][1]
        return (r, c) if 0 <= r < self.h and 0 <= c < self.w else s   # 벽이면 제자리

    def reward(self, s2):
        return {'G': GOAL_R, 'X': PIT_R}.get(GRID[s2[0]][s2[1]], STEP_R)

    def transitions(self, s, a):
        """(확률, 다음 상태) 목록 — 마르코프 성질: 다음 상태는 지금 상태와 행동만으로 정해진다."""
        side = [2, 3] if a in (0, 1) else [0, 1]
        return [(1 - self.slip, self.move(s, a))] + [(self.slip / 2, self.move(s, b)) for b in side]

    def step(self, s, a, rng):
        probs, nexts = zip(*self.transitions(s, a))
        s2 = nexts[rng.choice(len(nexts), p=probs)]
        return s2, self.reward(s2), self.terminal(s2)


def value_iteration(env, tol=1e-6):
    V = np.zeros((env.h, env.w))
    for it in range(1000):
        delta = 0.0
        for r in range(env.h):
            for c in range(env.w):
                s = (r, c)
                if env.terminal(s):
                    continue
                # 벨만 최적 방정식: V(s) = max_a Σ P(s'|s,a) [R + γ V(s')]
                q = [sum(p * (env.reward(s2) + GAMMA * (0 if env.terminal(s2) else V[s2]))
                         for p, s2 in env.transitions(s, a)) for a in range(4)]
                delta = max(delta, abs(max(q) - V[s]))
                V[s] = max(q)
        if delta < tol:
            return V, it + 1
    return V, it + 1


def q_learning(env, episodes=3000, alpha=0.1, eps=0.2, seed=0):
    rng = np.random.default_rng(seed)
    Q = np.zeros((env.h, env.w, 4))
    returns = []
    for ep in range(episodes):
        s, total, done, t = env.start, 0.0, False, 0
        while not done and t < 100:
            a = rng.integers(4) if rng.random() < eps else int(np.argmax(Q[s]))   # ε-탐욕 탐색
            s2, r, done = env.step(s, a, rng)
            # Q-Learning: 실제로 한 번 겪은 (s, a, r, s') 로 벨만 목표값 쪽으로 조금씩 이동
            target = r + (0 if done else GAMMA * Q[s2].max())
            Q[s][a] += alpha * (target - Q[s][a])
            s, total, t = s2, total + r, t + 1
        returns.append(total)
    return Q, returns


def draw(ax, values, policy, env, title):
    ax.imshow(values, cmap='RdYlGn', vmin=-1, vmax=1)
    for r in range(env.h):
        for c in range(env.w):
            ch = GRID[r][c]
            text = ch if ch in 'GX' else f'{values[r, c]:.2f}\n{ARROWS[policy[r, c]]}'
            ax.text(c, r, text, ha='center', va='center', fontsize=10)
    ax.set_title(title)
    ax.set_xticks([])
    ax.set_yticks([])


def main():
    env = GridWorld()
    V, iters = value_iteration(env)
    pi_vi = np.zeros((env.h, env.w), int)
    for r in range(env.h):
        for c in range(env.w):
            q = [sum(p * (env.reward(s2) + GAMMA * (0 if env.terminal(s2) else V[s2]))
                     for p, s2 in env.transitions((r, c), a)) for a in range(4)]
            pi_vi[r, c] = int(np.argmax(q))
    print(f'가치 반복: {iters}번 반복 후 수렴, 시작 칸 가치 V(S) = {V[env.start]:.3f}')

    Q, returns = q_learning(env)
    V_q, pi_q = Q.max(-1), Q.argmax(-1)
    free = [(r, c) for r in range(env.h) for c in range(env.w) if not env.terminal((r, c))]
    same = np.mean([pi_q[s] == pi_vi[s] for s in free])
    print(f'Q-Learning: 3000 에피소드 후 V(S) = {V_q[env.start]:.3f}, 가치 반복과 정책 일치율 {same * 100:.0f}%')

    fig, axes = plt.subplots(1, 2, figsize=(9, 4.2))
    draw(axes[0], V, pi_vi, env, 'Value Iteration (model known)')
    draw(axes[1], V_q, pi_q, env, 'Q-Learning (model-free)')
    plt.tight_layout()
    plt.savefig('c2_values.png', dpi=110)

    plt.figure(figsize=(6, 3.2))
    k = 100
    plt.plot(np.convolve(returns, np.ones(k) / k, 'valid'))
    plt.xlabel('episode')
    plt.ylabel(f'return ({k}-episode mean)')
    plt.title('Q-Learning learning curve')
    plt.tight_layout()
    plt.savefig('c2_learning.png', dpi=110)
    print('그림 저장: c2_values.png, c2_learning.png')


if __name__ == '__main__':
    main()
