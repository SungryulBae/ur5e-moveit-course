"""2교시 실습 2: FollowJointTrajectory 액션으로 관절을 직접 제어한다 (MoveIt 없음).

실행:
  ros2 launch edu_arm_description control.launch.py
  ros2 run day1_examples c2_joint_commander                       # 기본 시퀀스
  ros2 run day1_examples c2_joint_commander 30 -90 90 -90 -90 0   # 원하는 관절각(도)
"""
import sys

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from day1_examples.common import ARM_JOINTS, READY

ACTION = '/arm_controller/follow_joint_trajectory'


class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)

    def move(self, q_rad, seconds=3.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q_rad]
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [point]

        self.get_logger().info(f'목표(도): {np.round(np.degrees(q_rad), 1).tolist()}')
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error('목표가 거부되었습니다')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        code = result_future.result().result.error_code
        self.get_logger().info('도착' if code == 0 else f'실패 (error_code={code})')
        return code == 0


def main():
    rclpy.init()
    node = JointCommander()
    if not node.client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error(f'{ACTION} 액션 서버가 없습니다. control.launch.py를 먼저 실행하세요.')
        return

    if len(sys.argv) == 7:
        targets = [np.radians([float(v) for v in sys.argv[1:]])]
    else:
        targets = [
            np.array(READY),
            np.array(READY) + np.radians([45, 0, 0, 0, 0, 0]),   # 베이스만 회전
            np.array(READY) + np.radians([45, 30, -30, 0, 0, 0]),  # 어깨 · 팔꿈치
            np.array(READY),
        ]
    for q in targets:
        node.move(q)

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
