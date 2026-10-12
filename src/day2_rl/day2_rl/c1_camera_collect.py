"""1교시 실습 1: MuJoCo 가상 카메라로 데이터 수집

로봇(UR5e + 2F-85)이 큐브 위를 여러 자세로 지나가는 동안
정면 카메라 · 손목 카메라 이미지와 로봇 상태(관절 각도, 끝단 위치, 큐브 위치)를 함께 저장한다.

  $MUJOCO_PYTHON -m day2_rl.c1_camera_collect                 # 기본 60 장
  $MUJOCO_PYTHON -m day2_rl.c1_camera_collect --frames 120

결과: camera_data.npz
  front, wrist : (N, 240, 320, 3) uint8 이미지
  qpos         : (N, 6) 관절 각도
  ee, cube     : (N, 3) 끝단 · 큐브 위치 (m)
  K_front      : 정면 카메라의 내부 파라미터 행렬 (3x3) — 핀홀 카메라 모델
  T_front      : 정면 카메라 위치 · 자세 (4x4, 월드 기준)
"""
import argparse

import mujoco
import numpy as np

from day2_rl.envs import UR5eTaskEnv
from day2_rl.experts import EXPERTS

W, H = 320, 240


def intrinsics(model, cam, width=W, height=H):
    """MuJoCo 카메라의 세로 화각(fovy) → 핀홀 모델 내부 파라미터 K."""
    fovy = np.radians(model.cam_fovy[model.camera(cam).id])
    f = 0.5 * height / np.tan(fovy / 2)          # 초점 거리 (픽셀 단위)
    return np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1]])


def extrinsics(data, cam):
    """카메라 위치 · 자세 (월드 기준 4x4). MuJoCo 카메라는 -z 방향을 바라본다."""
    T = np.eye(4)
    T[:3, :3] = data.cam(cam).xmat.reshape(3, 3)
    T[:3, 3] = data.cam(cam).xpos
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=60)
    args = ap.parse_args()

    env = UR5eTaskEnv(task='pick', max_steps=10_000)
    renderer = mujoco.Renderer(env.model, H, W)
    rec = {k: [] for k in ['front', 'wrist', 'qpos', 'ee', 'cube']}
    obs, _ = env.reset(seed=0)
    qadr = [env.model.joint(n).qposadr[0] for n in
            ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
             'wrist_3_joint']]

    # 로봇 제어로 데이터 수집: 전문가가 큐브를 집어 들어 올리는 동안 + 무작위로 흔들기
    for i in range(args.frames):
        a = EXPERTS['pick'](obs)
        if i % 20 >= 15:
            a[:3] = env.np_random.uniform(-1, 1, 3)       # 가끔 무작위로 움직여 다양한 시점 확보
        obs, r, term, trunc, info = env.step(a)
        if term:
            obs, _ = env.reset(seed=i)
        for cam in ['front', 'wrist']:
            renderer.update_scene(env.data, camera=cam)
            rec[cam].append(renderer.render().copy())
        rec['qpos'].append(env.data.qpos[qadr].copy())
        rec['ee'].append(env.ee_pos())
        rec['cube'].append(env.cube_pos())
        if (i + 1) % 20 == 0:
            print(f'{i + 1}/{args.frames} 장 수집', flush=True)

    np.savez_compressed('camera_data.npz', **{k: np.array(v) for k, v in rec.items()},
                        K_front=intrinsics(env.model, 'front'), T_front=extrinsics(env.data, 'front'))
    print(f'저장: camera_data.npz  (이미지 {args.frames} 장 x 카메라 2대, {W}x{H})')


if __name__ == '__main__':
    main()
