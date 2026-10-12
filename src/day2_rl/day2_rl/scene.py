"""2일차 공통: MuJoCo 장면 만들기 — UR5e + Robotiq 2F-85 + 바닥(작업대) + 큐브 + 목표 표시 + 카메라 2대

로봇 모델은 MuJoCo Menagerie(공식 모델 모음)의 universal_robots_ur5e, robotiq_2f85 를 쓴다.
(setup/install.sh 가 third_party/mujoco_menagerie 에 내려받는다)
"""
import os

import mujoco
import numpy as np

ARM_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
HOME = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])   # 그리퍼가 바닥을 향한 준비 자세
TCP_SITE = 'gripper/pinch'          # 두 손가락 사이 (물체를 잡는 점)
GRIPPER_ACTUATOR = 'gripper/fingers_actuator'   # 0 = 완전 열림, 255 = 완전 닫힘
CUBE_SIZE = 0.02                    # 큐브 반 변 길이 (m) → 한 변 4 cm


def menagerie_dir():
    """MuJoCo Menagerie 위치: $MENAGERIE_DIR → <실습 폴더>/third_party → (강사 PC) instructor/third_party."""
    candidates = [os.environ.get('MENAGERIE_DIR', '')]
    if os.environ.get('COURSE_WS'):
        candidates.append(os.path.join(os.environ['COURSE_WS'], 'third_party', 'mujoco_menagerie'))
    candidates.append(os.path.expanduser('~/ur5e_course_ws/instructor/third_party/mujoco_menagerie'))
    for c in candidates:
        if c and os.path.isfile(os.path.join(c, 'universal_robots_ur5e', 'ur5e.xml')):
            return c
    raise FileNotFoundError('MuJoCo Menagerie 를 찾을 수 없습니다. setup/install.sh 를 다시 실행하세요.')


def build_spec(with_cube=True):
    root = menagerie_dir()
    arm = mujoco.MjSpec.from_file(os.path.join(root, 'universal_robots_ur5e', 'ur5e.xml'))
    gripper = mujoco.MjSpec.from_file(os.path.join(root, 'robotiq_2f85', '2f85.xml'))
    # 물체를 안정적으로 쥐기 위한 접촉 설정 (2F-85 모델 권장값) — 붙이기 전에 맞춰야 충돌 경고가 안 난다
    arm.option.timestep = 0.002
    arm.option.impratio = 10
    arm.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    arm.attach(gripper, site=arm.site('attachment_site'), prefix='gripper/')

    # 실제 UR 컨트롤러처럼 중력 보상을 켜고(팔이 무게로 처지지 않게), 관절 감쇠를 줄여 반응을 빠르게 한다
    # (공식 모델 그대로면 반응 시간이 약 0.2 초라 20 Hz 끝단 명령을 따라가지 못한다)
    for body in arm.bodies:
        if body.name and body.name not in ('world',):
            body.gravcomp = 1.0
    for act in arm.actuators:
        if act.name in ('shoulder_pan', 'shoulder_lift', 'elbow'):
            act.biasprm[2] = -150.0
        elif act.name in ('wrist_1', 'wrist_2', 'wrist_3'):
            act.biasprm[2] = -20.0

    world = arm.worldbody
    world.add_light(pos=[0, 0.5, 2.0], dir=[0, 0, -1], diffuse=[0.8, 0.8, 0.8])
    world.add_geom(name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1.5, 1.5, 0.05],
                   rgba=[0.75, 0.75, 0.72, 1], friction=[1.0, 0.005, 0.0001])
    # 작업 영역 표시 (충돌 없음)
    world.add_geom(name='workarea', type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.2, 0.15, 0.0005],
                   pos=[-0.1, 0.55, 0.0005], rgba=[0.9, 0.9, 0.85, 1], contype=0, conaffinity=0)

    # 목표 위치 표시 (충돌 없음, 눈으로만 보임)
    goal = world.add_body(name='goal', pos=[-0.1, 0.55, 0.1], mocap=True)
    goal.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.02, 0, 0], rgba=[0.1, 0.8, 0.2, 0.5],
                  contype=0, conaffinity=0)

    if with_cube:
        cube = world.add_body(name='cube', pos=[-0.1, 0.55, CUBE_SIZE])
        cube.add_freejoint(name='cube_joint')
        cube.add_geom(name='cube', type=mujoco.mjtGeom.mjGEOM_BOX, size=[CUBE_SIZE] * 3,
                      rgba=[0.85, 0.2, 0.2, 1], mass=0.05, friction=[1.0, 0.005, 0.0001])

    # 카메라: 작업대를 비스듬히 내려다보는 정면 카메라, 그리퍼에 붙은 손목 카메라
    world.add_camera(name='front', pos=[-0.1, 1.25, 0.6], xyaxes=[-1, 0, 0, 0, -0.55, 0.83], fovy=45)
    gripper_base = arm.body('gripper/base')
    # 손가락이 벌어지는 방향(그리퍼 y)과 수직인 쪽 뒤 · 위에서 손가락 끝 아래를 내려다본다 (손가락에 가리지 않게)
    gripper_base.add_camera(name='wrist', pos=[-0.09, 0.0, 0.065], quat=[-0.2108, 0.6749, 0.6749, -0.2108], fovy=75)
    return arm


def build_model(with_cube=True):
    model = build_spec(with_cube).compile()
    data = mujoco.MjData(model)
    reset_home(model, data)
    return model, data


def arm_qpos_ids(model):
    return np.array([model.joint(n).qposadr[0] for n in ARM_JOINTS])


def arm_dof_ids(model):
    return np.array([model.joint(n).dofadr[0] for n in ARM_JOINTS])


def reset_home(model, data):
    mujoco.mj_resetData(model, data)
    data.qpos[arm_qpos_ids(model)] = HOME
    data.ctrl[:6] = HOME
    data.actuator(GRIPPER_ACTUATOR).ctrl[0] = 0.0
    mujoco.mj_forward(model, data)
