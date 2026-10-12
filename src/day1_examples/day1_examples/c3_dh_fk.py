"""3교시 실습 1: DH 파라미터로 직접 계산한 FK와 ROS TF를 비교한다.

- /joint_states 로 현재 관절값을 받는다
- common.fk() (UR5e DH)로 base_link -> tool0 를 계산한다
- TF가 알려 주는 base_link -> tool0 와 위치 · 자세 오차를 출력한다

실행:
  ros2 launch edu_arm_description control.launch.py
  ros2 run day1_examples c3_dh_fk
  (다른 터미널에서 c2_joint_commander 로 관절을 움직여 보며 비교)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from day1_examples.common import fk, joint_positions_by_name, quat_to_rot


class DhFkCheck(Node):
    def __init__(self):
        super().__init__('dh_fk_check')
        self.q = None
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(1.0, self.on_timer)

    def on_joint_states(self, msg):
        self.q = joint_positions_by_name(msg)

    def on_timer(self):
        if self.q is None:
            return
        try:
            tf = self.buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
        except Exception:
            return
        T_dh = fk(self.q)
        t, r = tf.transform.translation, tf.transform.rotation
        p_tf = np.array([t.x, t.y, t.z])
        R_tf = quat_to_rot(r.x, r.y, r.z, r.w)

        pos_err_mm = np.linalg.norm(T_dh[:3, 3] - p_tf) * 1000
        # 두 회전의 차이 각도: R_err = R_dh^T R_tf 의 회전각
        cos_angle = (np.trace(T_dh[:3, :3].T @ R_tf) - 1) / 2
        rot_err_deg = np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))

        np.set_printoptions(precision=4, suppress=True)
        self.get_logger().info(
            f'\n  q(도)   = {np.degrees(self.q).round(1)}'
            f'\n  DH FK   = {T_dh[:3, 3]}'
            f'\n  TF      = {p_tf}'
            f'\n  오차    = 위치 {pos_err_mm:.3f} mm, 자세 {rot_err_deg:.3f}°')


def main():
    rclpy.init()
    node = DhFkCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
