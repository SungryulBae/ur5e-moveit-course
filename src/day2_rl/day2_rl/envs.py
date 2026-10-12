"""5교시: MuJoCo UR5e 강화학습 환경 — Reach / Push / Pick (Gymnasium 형식)

State (관찰, 16차원)
  끝단 위치(3) · 그리퍼 열림 정도(1) · 큐브 위치(3) · 목표 위치(3) · 큐브-끝단(3) · 목표-큐브(3)
  (Reach 는 큐브 대신 목표-끝단을 쓴다)
Action (행동)
  끝단을 x, y, z 로 얼마나 옮길지 [-1, 1] → 한 스텝 최대 2 cm  (+ Pick 은 그리퍼 열기/닫기 1개)
  → 내부에서 역기구학(IK)으로 관절 목표를 계산 (1일차 3교시 수치 IK 와 같은 원리)
Reward (보상)
  dense : 목표까지 거리가 줄수록 커짐 (-거리)
  sparse: 성공이면 0, 아니면 -1  (HIL-SERL 이 쓰는 방식 — 학습은 어렵지만 설계가 쉽다)
성공 / 실패
  성공하면 에피소드 종료(terminated), 50 스텝(2.5 초) 안에 못 하면 시간 초과(truncated)
"""
import gymnasium as gym
import mujoco
import numpy as np

from day2_rl.scene import (CUBE_SIZE, GRIPPER_ACTUATOR, TCP_SITE, arm_dof_ids, arm_qpos_ids,
                           build_model, reset_home)

CONTROL_DT = 0.05            # 행동 한 번 = 0.05 초 (20 Hz)
MAX_STEP = 0.02              # 한 스텝 최대 이동 (m)
WORKSPACE_LOW = np.array([-0.30, 0.38, 0.015])     # 끝단이 움직일 수 있는 상자 (m)
WORKSPACE_HIGH = np.array([0.10, 0.72, 0.30])


class EEController:
    """끝단(TCP) 목표 위치 → 관절 목표 (감쇠 최소제곱 IK). 그리퍼는 항상 바닥을 향하게 유지."""

    def __init__(self, model):
        self.model = model
        self.ik_data = mujoco.MjData(model)
        self.qadr = arm_qpos_ids(model)
        self.dadr = arm_dof_ids(model)
        self.site = model.site(TCP_SITE).id
        self.target_rot = None

    def solve(self, data, target_pos, iters=10, damping=1e-2):
        d = self.ik_data
        d.qpos[:] = data.qpos
        if self.target_rot is None:
            mujoco.mj_kinematics(self.model, d)
            self.target_rot = d.site_xmat[self.site].reshape(3, 3).copy()
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        for _ in range(iters):
            mujoco.mj_kinematics(self.model, d)
            mujoco.mj_comPos(self.model, d)
            pos = d.site_xpos[self.site]
            rot = d.site_xmat[self.site].reshape(3, 3)
            err_pos = target_pos - pos
            r_err = self.target_rot @ rot.T            # 자세 오차 (축-각 벡터)
            err_rot = 0.5 * np.array([r_err[2, 1] - r_err[1, 2], r_err[0, 2] - r_err[2, 0],
                                      r_err[1, 0] - r_err[0, 1]])
            err = np.concatenate([err_pos, err_rot])
            if np.linalg.norm(err_pos) < 1e-4 and np.linalg.norm(err_rot) < 1e-3:
                break
            mujoco.mj_jacSite(self.model, d, jacp, jacr, self.site)
            J = np.vstack([jacp, jacr])[:, self.dadr]
            dq = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(6), err)
            d.qpos[self.qadr] += dq
        return d.qpos[self.qadr].copy()


class UR5eTaskEnv(gym.Env):
    metadata = {'render_modes': ['rgb_array'], 'render_fps': int(1 / CONTROL_DT)}

    def __init__(self, task='reach', reward_type='dense', max_steps=50, render_mode=None, camera='front'):
        assert task in ('reach', 'push', 'pick')
        self.task, self.reward_type, self.max_steps = task, reward_type, max_steps
        self.model, self.data = build_model(with_cube=True)
        self.ctrl = EEController(self.model)
        self.n_substeps = int(round(CONTROL_DT / self.model.opt.timestep))
        self.tcp = self.model.site(TCP_SITE).id
        self.cube_qadr = self.model.joint('cube_joint').qposadr[0]
        self.goal_mocap = self.model.body('goal').mocapid[0]
        self.grip = self.model.actuator(GRIPPER_ACTUATOR).id
        self.finger_qadr = self.model.joint('gripper/right_driver_joint').qposadr[0]

        act_dim = 4 if task == 'pick' else 3
        self.action_space = gym.spaces.Box(-1.0, 1.0, (act_dim,), np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (16,), np.float32)
        self.render_mode, self.camera = render_mode, camera
        self._renderer = None
        self.np_random = np.random.default_rng()

    # ---------------- 상태 읽기 ----------------
    def ee_pos(self):
        return self.data.site_xpos[self.tcp].copy()

    def cube_pos(self):
        return self.data.qpos[self.cube_qadr:self.cube_qadr + 3].copy()

    def goal_pos(self):
        return self.data.mocap_pos[self.goal_mocap].copy()

    def gripper_opening(self):
        return 1.0 - np.clip(self.data.qpos[self.finger_qadr] / 0.8, 0.0, 1.0)   # 1 = 열림, 0 = 닫힘

    def _obs(self):
        ee, cube, goal = self.ee_pos(), self.cube_pos(), self.goal_pos()
        if self.task == 'reach':
            rel1, rel2 = goal - ee, np.zeros(3)
        else:
            rel1, rel2 = cube - ee, goal - cube
        return np.concatenate([ee, [self.gripper_opening()], cube, goal, rel1, rel2]).astype(np.float32)

    # ---------------- 성공 · 보상 ----------------
    def _distance(self):
        if self.task == 'reach':
            return np.linalg.norm(self.goal_pos() - self.ee_pos())
        if self.task == 'push':
            return np.linalg.norm((self.goal_pos() - self.cube_pos())[:2])
        return np.linalg.norm(self.goal_pos() - self.cube_pos())

    def is_success(self):
        return self._distance() < (0.03 if self.task == 'reach' else 0.04)

    def _reward(self, success):
        if self.reward_type == 'sparse':
            return 0.0 if success else -1.0
        ee, cube = self.ee_pos(), self.cube_pos()
        if self.task == 'reach':
            return -self._distance()
        return -self._distance() - 0.5 * np.linalg.norm(cube - ee)

    # ---------------- Gymnasium API ----------------
    def reset(self, seed=None, options=None):
        if seed is not None:
            self.np_random = np.random.default_rng(seed)
        rng = self.np_random
        reset_home(self.model, self.data)
        self.ctrl.target_rot = None
        start = np.array([-0.10, 0.55, 0.20]) + rng.uniform([-0.05, -0.05, -0.03], [0.05, 0.05, 0.03])
        self._move_arm_to(start)

        cube_xy = np.array([-0.10, 0.55]) + rng.uniform(-0.08, 0.08, 2)
        if self.task == 'reach':
            cube_xy = np.array([0.08, 0.40])            # 방해되지 않는 곳에 치워 둠
            goal = rng.uniform([-0.25, 0.42, 0.05], [0.05, 0.68, 0.25])
        elif self.task == 'push':
            while True:
                goal = np.append(np.array([-0.10, 0.55]) + rng.uniform(-0.12, 0.12, 2), CUBE_SIZE)
                if np.linalg.norm(goal[:2] - cube_xy) > 0.08:
                    break
        else:
            goal = np.append(cube_xy + rng.uniform(-0.05, 0.05, 2), rng.uniform(0.10, 0.18))
        self.data.qpos[self.cube_qadr:self.cube_qadr + 3] = [*cube_xy, CUBE_SIZE]
        self.data.qpos[self.cube_qadr + 3:self.cube_qadr + 7] = [1, 0, 0, 0]
        self.data.mocap_pos[self.goal_mocap] = goal
        self.data.actuator(GRIPPER_ACTUATOR).ctrl[0] = 255.0 if self.task == 'push' else 0.0
        mujoco.mj_forward(self.model, self.data)
        for _ in range(20):                               # 그리퍼 · 큐브 안정화
            mujoco.mj_step(self.model, self.data)
        self.steps = 0
        self.target = self.ee_pos()          # 끝단 목표 위치 (행동이 누적된다)
        return self._obs(), {'success': self.is_success()}

    def _move_arm_to(self, pos):
        q = self.ctrl.solve(self.data, pos, iters=100)
        self.data.qpos[arm_qpos_ids(self.model)] = q
        self.data.ctrl[:6] = q
        mujoco.mj_forward(self.model, self.data)

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        # 목표 위치를 누적해서 옮긴다. 팔이 늦게 따라와도 명령한 만큼 가도록 (단, 실제 위치에서 5 cm 이상 앞서지 않게)
        ee = self.ee_pos()
        target = np.clip(self.target + action[:3] * MAX_STEP, WORKSPACE_LOW, WORKSPACE_HIGH)
        self.target = ee + np.clip(target - ee, -0.05, 0.05)
        self.data.ctrl[:6] = self.ctrl.solve(self.data, self.target)
        if self.task == 'pick':
            self.data.ctrl[self.grip] = 255.0 if action[3] > 0 else 0.0     # +: 닫기, -: 열기
        for _ in range(self.n_substeps):
            mujoco.mj_step(self.model, self.data)
        self.steps += 1
        success = self.is_success()
        reward = self._reward(success)
        terminated = success
        truncated = self.steps >= self.max_steps
        return self._obs(), reward, terminated, truncated, {'success': success}

    def render(self, width=320, height=240, camera=None):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height, width)
        self._renderer.update_scene(self.data, camera=camera or self.camera)
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
