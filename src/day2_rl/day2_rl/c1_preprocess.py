"""1교시 실습 2: 전처리 기법과 시각화 (c1_camera_collect 로 만든 camera_data.npz 사용)

  $MUJOCO_PYTHON -m day2_rl.c1_preprocess

만드는 그림
  c1_samples.png      : 정면 · 손목 카메라 이미지 견본
  c1_preprocess.png   : 원본 → 자르기(crop) → 줄이기(resize) → 흑백 → 정규화 → 밝기 증강 → 무작위 이동 증강
  c1_projection.png   : 핀홀 카메라 모델로 큐브의 3D 위치를 이미지 픽셀로 투영 → 실제 이미지 위에 표시
  c1_trajectory.png   : 수집 중 끝단 · 큐브 위치 그래프
"""
import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


# ---------------- 전처리 함수 (numpy 만 사용) ----------------
def center_crop(img, size):
    h, w = img.shape[:2]
    top, left = (h - size) // 2, (w - size) // 2
    return img[top:top + size, left:left + size]


def resize_nearest(img, out_h, out_w):
    h, w = img.shape[:2]
    rows = (np.arange(out_h) * h / out_h).astype(int)
    cols = (np.arange(out_w) * w / out_w).astype(int)
    return img[rows][:, cols]


def to_gray(img):
    return (img[..., :3] @ np.array([0.299, 0.587, 0.114])).astype(np.float32)


def normalize(img):
    """0~255 정수 → 평균 0, 표준편차 1 근처의 실수 (신경망 입력용)."""
    x = img.astype(np.float32) / 255.0
    return (x - x.mean()) / (x.std() + 1e-6)


def brightness_jitter(img, rng, scale=0.4):
    return np.clip(img.astype(np.float32) * rng.uniform(1 - scale, 1 + scale), 0, 255).astype(np.uint8)


def random_shift(img, rng, pad=8):
    """테두리를 늘린 뒤 무작위 위치에서 잘라 내기 — 로봇 학습(DrQ 등)에서 자주 쓰는 증강."""
    h, w = img.shape[:2]
    padded = np.pad(img, ((pad, pad), (pad, pad), (0, 0)), mode='edge')
    dy, dx = rng.integers(0, 2 * pad + 1, 2)
    return padded[dy:dy + h, dx:dx + w]


def project(K, T_cam, p_world):
    """3D 점(월드) → 픽셀 (u, v). MuJoCo 카메라는 -z 를 보고, 이미지 y 는 아래로 증가한다."""
    p_cam = T_cam[:3, :3].T @ (p_world - T_cam[:3, 3])
    x, y, z = p_cam[0], -p_cam[1], -p_cam[2]          # MuJoCo(OpenGL) → 컴퓨터 비전 좌표계
    u = K[0, 0] * x / z + K[0, 2]
    v = K[1, 1] * y / z + K[1, 2]
    return u, v


def main():
    d = np.load('camera_data.npz')
    front, wrist = d['front'], d['wrist']
    rng = np.random.default_rng(0)
    n = len(front)
    print(f'이미지 {n} 장, 크기 {front.shape[1:]}, 자료형 {front.dtype}, 값 범위 {front.min()}~{front.max()}')

    # 1) 견본
    idx = np.linspace(0, n - 1, 4).astype(int)
    fig, axes = plt.subplots(2, 4, figsize=(12, 5))
    for j, i in enumerate(idx):
        axes[0, j].imshow(front[i])
        axes[0, j].set_title(f'front #{i}')
        axes[1, j].imshow(wrist[i])
        axes[1, j].set_title(f'wrist #{i}')
    for ax in axes.flat:
        ax.axis('off')
    plt.tight_layout()
    plt.savefig('c1_samples.png', dpi=100)

    # 2) 전처리 단계
    img = front[n // 2]
    crop = center_crop(img, 200)
    small = resize_nearest(crop, 84, 84)
    gray = to_gray(small)
    norm = normalize(small)
    bright = brightness_jitter(small, rng)
    shifted = random_shift(small, rng)
    steps = [(img, 'original 240x320'), (crop, 'center crop 200'), (small, 'resize 84x84'),
             (gray, 'grayscale'), (norm[..., 0], 'normalized (R)'), (bright, 'brightness aug'),
             (shifted, 'random shift aug')]
    fig, axes = plt.subplots(1, len(steps), figsize=(18, 3))
    for ax, (im, title) in zip(axes, steps):
        ax.imshow(im, cmap='gray' if im.ndim == 2 else None)
        ax.set_title(title, fontsize=9)
        ax.axis('off')
    plt.tight_layout()
    plt.savefig('c1_preprocess.png', dpi=100)
    print(f'정규화 후: 평균 {norm.mean():+.3f}, 표준편차 {norm.std():.3f}')

    # 3) 핀홀 모델 투영: 큐브 3D 위치 → 정면 이미지 픽셀
    fig, axes = plt.subplots(1, 4, figsize=(14, 3.2))
    for ax, i in zip(axes, idx):
        u, v = project(d['K_front'], d['T_front'], d['cube'][i])
        ax.imshow(front[i])
        ax.plot(u, v, 'c+', markersize=18, markeredgewidth=2)
        ax.set_title(f'#{i} cube -> ({u:.0f}, {v:.0f}) px')
        ax.axis('off')
    plt.tight_layout()
    plt.savefig('c1_projection.png', dpi=100)
    print('K (정면 카메라):\n', np.round(d['K_front'], 1))

    # 4) 로봇 상태 그래프
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.2))
    for k, name in enumerate('xyz'):
        axes[0].plot(d['ee'][:, k], label=f'ee {name}')
        axes[1].plot(d['cube'][:, k], label=f'cube {name}')
    for ax, t in zip(axes, ['end-effector position (m)', 'cube position (m)']):
        ax.set_title(t)
        ax.set_xlabel('frame')
        ax.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig('c1_trajectory.png', dpi=100)
    print('그림 저장: c1_samples.png, c1_preprocess.png, c1_projection.png, c1_trajectory.png')


if __name__ == '__main__':
    main()
