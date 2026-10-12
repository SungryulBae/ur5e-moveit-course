"""6교시: ROS 2 ↔ MuJoCo 브리지 — edu_arm URDF를 MuJoCo로 시뮬레이션하고 ROS 2와 관절 데이터를 주고받는다.

하는 일
  1. URDF(robot_description) → MuJoCo 모델로 변환하고 관절마다 위치 제어 액추에이터를 붙인다
  2. 물리 시뮬레이션을 실시간으로 돌리며 관절 상태를 /joint_states 로 내보낸다      (MuJoCo → ROS 2)
  3. 컨트롤러와 같은 이름의 액션 서버를 연다                                         (ROS 2 → MuJoCo)
       /arm_controller/follow_joint_trajectory  (팔 궤적)
       /gripper_controller/gripper_cmd          (그리퍼)
     → MoveIt 은 ros2_control 대신 이 노드에 그대로 명령을 보낸다

실행 (MuJoCo 가 설치된 가상환경의 파이썬으로):
  $MUJOCO_PYTHON -m day1_examples.c6_mujoco_bridge                         # 뷰어 창 표시
  $MUJOCO_PYTHON -m day1_examples.c6_mujoco_bridge --ros-args -p viewer:=false
  (보통은 edu_arm_moveit_config demo.launch.py sim:=mujoco 가 자동으로 실행한다)
"""
import re
import subprocess
import threading
import time

import mujoco
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from day1_examples.common import ARM_JOINTS

FINGER_JOINT = 'left_finger_joint'
MIMIC_JOINT = 'right_finger_joint'

# 관절별 (위치 제어 강성 kp [Nm/rad], 감쇠 [Nm·s/rad])
# URDF 의 토크 한계(어깨 · 팔꿈치 150 Nm, 손목 28 Nm)가 MuJoCo 에도 그대로 적용된다.
# 감쇠가 크면 "토크 한계 ÷ 감쇠" 가 최고 속도가 되므로, 손목은 감쇠를 작게 둔다
# (처음에 손목 감쇠를 50 으로 했더니 28 / 50 = 0.56 rad/s 로 묶여 MoveIt 궤적을 못 따라갔다).
GAINS = {
    'shoulder_pan_joint': (3000.0, 20.0),
    'shoulder_lift_joint': (3000.0, 20.0),
    'elbow_joint': (3000.0, 20.0),
    'wrist_1_joint': (800.0, 4.0),
    'wrist_2_joint': (800.0, 4.0),
    'wrist_3_joint': (800.0, 4.0),
    'left_finger_joint': (500.0, 20.0),
    'right_finger_joint': (500.0, 20.0),
}


def load_urdf():
    """edu_arm xacro → URDF 문자열 (MuJoCo 가 모르는 태그는 제거)."""
    xacro_file = get_package_share_directory('edu_arm_description') + '/urdf/edu_arm.urdf.xacro'
    urdf = subprocess.run(['xacro', xacro_file, 'sim_mode:=mock'],
                          check=True, capture_output=True, text=True).stdout
    urdf = re.sub(r'<ros2_control.*?</ros2_control>', '', urdf, flags=re.S)
    urdf = re.sub(r'<gazebo.*?</gazebo>', '', urdf, flags=re.S)
    # URDF 의 'world' 링크는 MuJoCo 의 worldbody 와 겹치므로 제거 (뿌리 링크는 자동으로 바닥에 고정된다)
    urdf = re.sub(r'<link name="world"\s*/>', '', urdf)
    urdf = re.sub(r'<joint name="world_joint".*?</joint>', '', urdf, flags=re.S)
    # 고정 링크를 합치지 않고, 시각 형상도 남긴다
    urdf = urdf.replace('<robot name="edu_arm">',
                        '<robot name="edu_arm"><mujoco><compiler fusestatic="false" '
                        'discardvisual="false"/></mujoco>', 1)
    return urdf


def build_model(urdf, initial, return_spec=False):
    """URDF → MjSpec → 바닥 · 조명 · 액추에이터 추가 → MjModel."""
    spec = mujoco.MjSpec.from_string(urdf)
    spec.option.timestep = 0.002

    world = spec.worldbody
    world.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1],
                   rgba=[0.85, 0.85, 0.85, 1], pos=[0, 0, -0.001])
    world.add_light(pos=[0, 0, 3], dir=[0, 0, -1])

    for j in spec.joints:
        if j.name not in GAINS:
            continue
        kp, damping = GAINS[j.name]
        j.damping = [damping, 0.0, 0.0]      # MuJoCo 3.15: 감쇠 계수는 3개짜리 배열 (첫 값이 선형 감쇠)
        if j.name == MIMIC_JOINT:
            continue  # URDF 의 <mimic> 이 등식 구속으로 변환되어 왼쪽 손가락을 따라간다
        act = spec.add_actuator(name=j.name, target=j.name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.set_to_position(kp=kp)

    model = spec.compile()
    data = mujoco.MjData(model)
    for name, q in initial.items():
        data.joint(name).qpos[0] = q
        data.actuator(name).ctrl[0] = q
    mujoco.mj_forward(model, data)
    if return_spec:
        return model, data, spec
    return model, data


class MujocoBridge(Node):
    def __init__(self):
        super().__init__('mujoco_bridge')
        self.declare_parameter('viewer', True)
        self.declare_parameter('publish_rate', 100.0)

        initial = dict(zip(ARM_JOINTS, [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]))
        initial[FINGER_JOINT] = 0.0
        self.model, self.data = build_model(load_urdf(), initial)
        self.lock = threading.Lock()
        self.joint_names = ARM_JOINTS + [FINGER_JOINT, MIMIC_JOINT]
        self.log = self.get_logger()
        self.log.info(f'MuJoCo 모델 준비: 관절 {self.model.njnt}개, 액추에이터 {self.model.nu}개')

        # 팔 궤적 실행 상태 (시뮬레이션 루프가 매 스텝 읽는다)
        self.traj = None          # (시작 시각, [t...], [[q...]...])

        cb = ReentrantCallbackGroup()
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / self.get_parameter('publish_rate').value, self.publish_joint_states,
                          callback_group=cb)
        ActionServer(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
                     execute_callback=self.execute_trajectory, goal_callback=self.accept_trajectory,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=cb)
        ActionServer(self, GripperCommand, '/gripper_controller/gripper_cmd',
                     execute_callback=self.execute_gripper, callback_group=cb)

        self.running = True
        self.sim_thread = threading.Thread(target=self.simulate, daemon=True)
        self.sim_thread.start()

    # ---------------- 물리 시뮬레이션 (실시간) ----------------
    def simulate(self):
        viewer = None
        if self.get_parameter('viewer').value:
            from mujoco import viewer as mj_viewer    # 화면이 있을 때만 불러온다
            viewer = mj_viewer.launch_passive(self.model, self.data)
        dt = self.model.opt.timestep
        next_time = time.monotonic()
        while self.running and rclpy.ok():
            with self.lock:
                self.apply_trajectory()
                mujoco.mj_step(self.model, self.data)
            if viewer is not None:
                if not viewer.is_running():
                    viewer = None
                else:
                    viewer.sync()
            next_time += dt
            sleep = next_time - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_time = time.monotonic()   # 계산이 밀리면 따라잡지 않고 다시 맞춘다

    def apply_trajectory(self):
        """현재 시각에 해당하는 궤적 위치를 선형 보간해서 팔 액추에이터 목표로 넣는다."""
        if self.traj is None:
            return
        t0, times, points = self.traj
        t = time.monotonic() - t0
        if t >= times[-1]:
            q = points[-1]
        else:
            k = np.searchsorted(times, t)
            if k == 0:
                q = points[0]
            else:
                a = (t - times[k - 1]) / (times[k] - times[k - 1])
                q = (1 - a) * points[k - 1] + a * points[k]
        for name, value in zip(ARM_JOINTS, q):
            self.data.actuator(name).ctrl[0] = value

    # ---------------- MuJoCo → ROS 2 ----------------
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.name = list(self.joint_names)
            msg.position = [float(self.data.joint(n).qpos[0]) for n in self.joint_names]
            msg.velocity = [float(self.data.joint(n).qvel[0]) for n in self.joint_names]
        self.js_pub.publish(msg)

    def arm_positions(self):
        with self.lock:
            return np.array([self.data.joint(n).qpos[0] for n in ARM_JOINTS])

    # ---------------- ROS 2 → MuJoCo: 팔 ----------------
    def accept_trajectory(self, goal):
        names = list(goal.trajectory.joint_names)
        if sorted(names) != sorted(ARM_JOINTS):
            self.log.error(f'모르는 관절 이름: {names}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_trajectory(self, goal_handle):
        jt = goal_handle.request.trajectory
        order = [list(jt.joint_names).index(n) for n in ARM_JOINTS]   # 이름 순서 맞추기
        times = [0.0] + [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in jt.points]
        points = [self.arm_positions()] + [np.array(p.positions)[order] for p in jt.points]
        times, points = np.array(times), np.array(points)
        self.log.info(f'궤적 실행: 점 {len(jt.points)}개, {times[-1]:.2f}초')

        self.traj = (time.monotonic(), times, points)
        deadline = time.monotonic() + times[-1] + 2.0      # 끝난 뒤 2초 안에 목표 근처로 와야 한다
        result = FollowJointTrajectory.Result()
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                err = np.degrees(self.arm_positions() - points[-1])
                self.log.warn(f'궤적 취소됨 (경과 {time.monotonic() - self.traj[0]:.2f}s / {times[-1]:.2f}s), '
                              f'남은 오차(도): {np.round(err, 2).tolist()}')
                self.traj = None
                goal_handle.canceled()
                return result
            done = time.monotonic() - self.traj[0] >= times[-1]
            error = np.max(np.abs(self.arm_positions() - points[-1]))
            if done and error < 0.02:     # 중력으로 약간 처지므로 0.02 rad(약 1.1°) 허용
                goal_handle.succeed()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result
            time.sleep(0.01)
        self.log.warn(f'목표 도달 실패: 최대 오차 {np.degrees(error):.2f}°')
        goal_handle.abort()
        result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
        return result

    # ---------------- ROS 2 → MuJoCo: 그리퍼 ----------------
    def execute_gripper(self, goal_handle):
        target = float(goal_handle.request.command.position)
        with self.lock:
            self.data.actuator(FINGER_JOINT).ctrl[0] = target
        result = GripperCommand.Result()
        start = time.monotonic()
        last = None
        while time.monotonic() - start < 3.0:
            time.sleep(0.05)
            with self.lock:
                pos = float(self.data.joint(FINGER_JOINT).qpos[0])
            if abs(pos - target) < 0.002:
                result.reached_goal = True
                break
            if last is not None and abs(pos - last) < 1e-5 and time.monotonic() - start > 0.5:
                result.stalled = True      # 물체를 잡아 더 못 닫힘
                break
            last = pos
        result.position = pos
        goal_handle.succeed()
        return result

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main():
    rclpy.init()
    node = MujocoBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
