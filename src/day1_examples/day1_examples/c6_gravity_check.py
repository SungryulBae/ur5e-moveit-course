"""6교시 실습 3: mock vs MuJoCo — "명령한 자세"와 "실제 자세"의 차이 보기

같은 코드를 mock 과 MuJoCo 에서 실행해 비교한다.
  - mock   : 명령한 값이 곧 상태 → 오차 0
  - MuJoCo : 무게 · 중력 · 토크 한계가 있음 → 어깨 · 팔꿈치가 조금 처진다

실행:
  ros2 launch edu_arm_moveit_config demo.launch.py              (mock)    또는
  ros2 launch edu_arm_moveit_config demo.launch.py sim:=mujoco  (MuJoCo)
  ros2 run day1_examples c6_gravity_check
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from day1_examples.c2_joint_commander import JointCommander
from day1_examples.common import ARM_JOINTS, joint_positions_by_name

# 팔을 수평으로 길게 뻗을수록 어깨에 걸리는 중력 토크가 커진다
POSES = {
    '준비 자세 (팔 접음)': [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
    '앞으로 뻗음': [0.0, -0.6, 0.6, -1.5708, -1.5708, 0.0],
    '수평으로 쭉 뻗음 (영점)': [0.0, 0.0, 0.0, -1.5708, -1.5708, 0.0],
}


def main():
    rclpy.init()
    commander = JointCommander()
    if not commander.client.wait_for_server(timeout_sec=10.0):
        commander.get_logger().error('팔 액션 서버가 없습니다. demo.launch.py 를 먼저 실행하세요.')
        return
    reader = Node('gravity_check')
    state = {}
    reader.create_subscription(JointState, '/joint_states',
                               lambda m: state.update(q=joint_positions_by_name(m)), 10)

    for label, target in POSES.items():
        commander.move(np.array(target), seconds=3.0)
        t = time.monotonic()
        while time.monotonic() - t < 1.0:          # 1초 기다린 뒤 측정 (정착)
            rclpy.spin_once(reader, timeout_sec=0.05)
        err = np.degrees(state['q'] - np.array(target))
        text = '  '.join(f'{n.replace("_joint", "")}={e:+.2f}°' for n, e in zip(ARM_JOINTS, err))
        reader.get_logger().info(f'[{label}] 명령 - 실제 오차: {text}')

    commander.destroy_node()
    reader.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
