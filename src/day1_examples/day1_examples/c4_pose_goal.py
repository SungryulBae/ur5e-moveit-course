"""4교시 실습 1: Pose -> IK -> 경로 계획 -> Trajectory 실행 (MoveIt 2)

move_group 하나에 "끝단을 이 자세로" 라고 요청하면, MoveIt 이
  ① IK 로 목표 관절값을 찾고 ② 충돌 없는 경로를 계획하고 ③ 시간 정보를 붙인 궤적을 ④ 컨트롤러로 실행한다.

실행:
  ros2 launch edu_arm_moveit_config demo.launch.py
  ros2 run day1_examples c4_pose_goal
"""
import rclpy
from rclpy.node import Node

from day1_examples.common import READY
from day1_examples.moveit_helper import MoveItHelper, make_pose


def main():
    rclpy.init()
    node = Node('c4_pose_goal')
    arm = MoveItHelper(node)
    if not arm.wait_for_servers():
        return
    arm.clear_paths()   # RViz 에 남아 있던 이전 끝단 경로 지우기 (주황 = 관절 공간 계획, 초록 = 직선)

    arm.move_to_joints(READY, '준비 자세')

    targets = [
        ('앞쪽 아래', make_pose(-0.45, -0.20, 0.25)),
        ('왼쪽', make_pose(-0.20, -0.45, 0.30)),
        ('작업 영역 밖 (실패 예시)', make_pose(-1.20, 0.0, 0.30)),
    ]
    for label, pose in targets:
        arm.move_to_pose(pose, label)

    arm.move_to_joints(READY, '준비 자세 복귀')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
