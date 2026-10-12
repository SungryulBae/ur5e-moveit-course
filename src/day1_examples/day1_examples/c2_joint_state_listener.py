"""2교시 실습 1: /joint_states 토픽 구독 — 관절 각도를 도(°) 단위로 출력한다.

실행:
  ros2 launch edu_arm_description control.launch.py
  ros2 run day1_examples c2_joint_state_listener
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from day1_examples.common import ARM_JOINTS, joint_positions_by_name


class JointStateListener(Node):
    def __init__(self):
        super().__init__('joint_state_listener')
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.count = 0

    def on_joint_states(self, msg):
        self.count += 1
        if self.count % 50:  # 100Hz 중 0.5초마다 한 번만 출력
            return
        q = joint_positions_by_name(msg)
        text = '  '.join(f'{n.replace("_joint", "")}={np.degrees(v):7.1f}°'
                         for n, v in zip(ARM_JOINTS, q))
        self.get_logger().info(text)


def main():
    rclpy.init()
    node = JointStateListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
