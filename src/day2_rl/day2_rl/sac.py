"""3교시: SAC(Soft Actor-Critic) 직접 구현 — 6교시에는 같은 코드에 RLPD 옵션을 켜서 쓴다

구성 요소
  Actor  (정책)      : 상태 → 행동의 평균 · 표준편차 → 샘플 → tanh 로 [-1, 1] 범위
  Critic (Q 함수 2개): (상태, 행동) → "앞으로 받을 보상 합"의 추정값. 둘 중 작은 값을 써서 과대평가를 막는다
  Target Critic      : Critic 을 천천히 따라가는 복사본 (학습 목표값을 안정시킨다)
  Replay Buffer      : 지나간 경험을 모아 두고 무작위로 꺼내 학습 (Off-Policy)
  Entropy 온도 α     : 행동의 무작위성(탐색)을 얼마나 장려할지, 목표 엔트로피에 맞춰 자동 조절

RLPD 옵션 (6교시, HIL-SERL 의 학습 방식)
  - offline 버퍼(시연 · 개입 데이터)와 online 버퍼(에이전트 경험)를 따로 두고 한 배치에 반반 섞는다
  - Critic 에 LayerNorm 을 넣어 학습을 안정시킨다
  - utd(update-to-data): 환경 한 스텝마다 여러 번 학습해 데이터를 알뜰하게 쓴다
"""
import copy

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

torch.set_num_threads(4)   # 강의실 CPU 를 독점하지 않도록


def mlp(inp, out, hidden=256, layer_norm=False):
    layers, d = [], inp
    for _ in range(2):
        layers.append(nn.Linear(d, hidden))
        if layer_norm:
            layers.append(nn.LayerNorm(hidden))
        layers.append(nn.ReLU())
        d = hidden
    layers.append(nn.Linear(d, out))
    return nn.Sequential(*layers)


class Actor(nn.Module):
    def __init__(self, obs_dim, act_dim, hidden=256):
        super().__init__()
        self.net = mlp(obs_dim, 2 * act_dim, hidden)

    def forward(self, obs, deterministic=False):
        mean, log_std = self.net(obs).chunk(2, dim=-1)
        log_std = log_std.clamp(-5, 2)
        std = log_std.exp()
        u = mean if deterministic else mean + std * torch.randn_like(mean)   # 재매개변수화
        a = torch.tanh(u)
        # tanh 로 눌러 담은 만큼 확률 밀도 보정
        log_prob = (-0.5 * ((u - mean) / std) ** 2 - log_std - 0.5 * np.log(2 * np.pi)).sum(-1)
        log_prob -= torch.log(1 - a.pow(2) + 1e-6).sum(-1)
        return a, log_prob


class Critic(nn.Module):
    """Q 함수 2개를 한 번에."""

    def __init__(self, obs_dim, act_dim, hidden=256, layer_norm=False):
        super().__init__()
        self.q1 = mlp(obs_dim + act_dim, 1, hidden, layer_norm)
        self.q2 = mlp(obs_dim + act_dim, 1, hidden, layer_norm)

    def forward(self, obs, act):
        x = torch.cat([obs, act], dim=-1)
        return self.q1(x).squeeze(-1), self.q2(x).squeeze(-1)


class ReplayBuffer:
    def __init__(self, obs_dim, act_dim, size=200_000):
        self.obs = np.zeros((size, obs_dim), np.float32)
        self.act = np.zeros((size, act_dim), np.float32)
        self.rew = np.zeros(size, np.float32)
        self.next_obs = np.zeros((size, obs_dim), np.float32)
        self.done = np.zeros(size, np.float32)       # 성공으로 끝남(terminated) = 1, 시간 초과는 0
        self.size, self.ptr, self.max = 0, 0, size

    def add(self, o, a, r, o2, d):
        i = self.ptr
        self.obs[i], self.act[i], self.rew[i], self.next_obs[i], self.done[i] = o, a, r, o2, d
        self.ptr = (self.ptr + 1) % self.max
        self.size = min(self.size + 1, self.max)

    def sample(self, n, rng):
        idx = rng.integers(0, self.size, n)
        return [self.obs[idx], self.act[idx], self.rew[idx], self.next_obs[idx], self.done[idx]]

    def __len__(self):
        return self.size


class SAC:
    def __init__(self, obs_dim, act_dim, hidden=256, lr=3e-4, gamma=0.99, tau=0.005, batch=256,
                 layer_norm=False, utd=1, seed=0):
        torch.manual_seed(seed)
        self.rng = np.random.default_rng(seed)
        self.actor = Actor(obs_dim, act_dim, hidden)
        self.critic = Critic(obs_dim, act_dim, hidden, layer_norm)
        self.critic_target = copy.deepcopy(self.critic)
        self.log_alpha = torch.zeros(1, requires_grad=True)
        self.target_entropy = -float(act_dim)
        self.opt_actor = torch.optim.Adam(self.actor.parameters(), lr=lr)
        self.opt_critic = torch.optim.Adam(self.critic.parameters(), lr=lr)
        self.opt_alpha = torch.optim.Adam([self.log_alpha], lr=lr)
        self.gamma, self.tau, self.batch, self.utd = gamma, tau, batch, utd
        self.online = ReplayBuffer(obs_dim, act_dim)
        self.offline = ReplayBuffer(obs_dim, act_dim)   # RLPD: 시연 · 개입 데이터

    @property
    def alpha(self):
        return self.log_alpha.exp().item()

    def act(self, obs, deterministic=False):
        with torch.no_grad():
            a, _ = self.actor(torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0), deterministic)
        return a.squeeze(0).numpy()

    def _batch(self):
        """offline 데이터가 있으면 반반 섞는다 (RLPD 의 symmetric sampling)."""
        if len(self.offline) > 0 and len(self.online) > 0:
            half = self.batch // 2
            a = self.online.sample(half, self.rng)
            b = self.offline.sample(self.batch - half, self.rng)
            parts = [np.concatenate([x, y]) for x, y in zip(a, b)]
        elif len(self.offline) > 0:
            parts = self.offline.sample(self.batch, self.rng)
        else:
            parts = self.online.sample(self.batch, self.rng)
        return [torch.as_tensor(p) for p in parts]

    def update(self):
        info = {}
        for _ in range(self.utd):
            o, a, r, o2, d = self._batch()
            # ---- Critic: 벨만 방정식 목표값 r + γ (min Q' - α log π) 에 맞춘다
            with torch.no_grad():
                a2, logp2 = self.actor(o2)
                q1t, q2t = self.critic_target(o2, a2)
                target = r + self.gamma * (1 - d) * (torch.min(q1t, q2t) - self.alpha * logp2)
            q1, q2 = self.critic(o, a)
            critic_loss = F.mse_loss(q1, target) + F.mse_loss(q2, target)
            self.opt_critic.zero_grad()
            critic_loss.backward()
            self.opt_critic.step()
            # ---- Target Critic: 천천히 따라간다
            with torch.no_grad():
                for p, pt in zip(self.critic.parameters(), self.critic_target.parameters()):
                    pt.mul_(1 - self.tau).add_(self.tau * p)

        # ---- Actor: Q 는 크게, 엔트로피(무작위성)도 적당히 크게
        a_new, logp = self.actor(o)
        q1, q2 = self.critic(o, a_new)
        actor_loss = (self.alpha * logp - torch.min(q1, q2)).mean()
        self.opt_actor.zero_grad()
        actor_loss.backward()
        self.opt_actor.step()
        # ---- α: 엔트로피가 목표보다 낮으면 키우고, 높으면 줄인다
        alpha_loss = -(self.log_alpha * (logp.detach() + self.target_entropy)).mean()
        self.opt_alpha.zero_grad()
        alpha_loss.backward()
        self.opt_alpha.step()
        info.update(critic_loss=critic_loss.item(), actor_loss=actor_loss.item(), alpha=self.alpha,
                    entropy=-logp.mean().item())
        return info

    def save(self, path):
        torch.save({'actor': self.actor.state_dict(), 'critic': self.critic.state_dict()}, path)

    def load(self, path):
        ck = torch.load(path, map_location='cpu')
        self.actor.load_state_dict(ck['actor'])
        self.critic.load_state_dict(ck['critic'])
