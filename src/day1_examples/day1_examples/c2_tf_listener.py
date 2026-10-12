"""2교시 실습 3: TF로 끝단(tool0) 위치 · 자세를 읽는다.

실행:
  ros2 launch edu_arm_description control.launch.py
  ros2 run day1_examples c2_tf_listener              # base_link -> tool0
  ros2 run day1_examples c2_tf_listener world tcp    # 다른 좌표계 쌍
"""
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from day1_examples.common import quat_to_rot, rot_to_rpy


class TfListener(Node):
    def __init__(self, parent, child):
        super().__init__('tf_listener')
        self.parent, self.child = parent, child
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            tf = self.buffer.lookup_transform(self.parent, self.child, rclpy.time.Time())
        except Exception as e:  # 아직 TF가 안 들어왔을 때
            self.get_logger().warn(f'TF 대기 중: {e}')
            return
        t, r = tf.transform.translation, tf.transform.rotation
        rpy = np.degrees(rot_to_rpy(quat_to_rot(r.x, r.y, r.z, r.w)))
        self.get_logger().info(
            f'{self.parent} -> {self.child}  '
            f'xyz=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) m  '
            f'rpy=({rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f})°  '
            f'quat=({r.x:.3f}, {r.y:.3f}, {r.z:.3f}, {r.w:.3f})')


def main():
    rclpy.init()
    parent, child = (sys.argv[1], sys.argv[2]) if len(sys.argv) >= 3 else ('base_link', 'tool0')
    node = TfListener(parent, child)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
