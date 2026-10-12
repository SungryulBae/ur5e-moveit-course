"""4교시 실습 2: 협동로봇의 직선 운동 + 그리퍼 열기 · 닫기

물체를 집는 동작의 기본 패턴:
  접근 위치(위) --직선 하강--> 파지 위치 --그리퍼 닫기--> --직선 상승--> 이동 --직선 하강--> 그리퍼 열기 --직선 상승

실행:
  ros2 launch edu_arm_moveit_config demo.launch.py
  ros2 run day1_examples c4_linear_gripper
"""
import copy

import rclpy
from rclpy.node import Node

from day1_examples.common import READY
from day1_examples.moveit_helper import MoveItHelper, make_pose

APPROACH_HEIGHT = 0.15   # 물체 위 접근 높이 (m)


def lowered(pose, dz):
    p = copy.deepcopy(pose)
    p.position.z -= dz
    return p


def main():
    rclpy.init()
    node = Node('c4_linear_gripper')
    arm = MoveItHelper(node)
    if not arm.wait_for_servers():
        return
    arm.clear_paths()   # RViz 에 남아 있던 이전 끝단 경로 지우기 (주황 = 관절 공간 계획, 초록 = 직선)

    # 손가락 끝은 tool0 보다 0.11 m 아래에 있다 (그리퍼 몸통 0.04 + 손가락 0.07).
    # 집기 높이 tool0 = 0.30 - 0.15 = 0.15 m → 손가락 끝 0.04 m (바닥 위 4 cm).
    # (처음에 0.25 로 잡았을 때는 손가락 끝이 바닥 아래로 내려갔다 — mock 은 바닥이 없어 몰랐고 MuJoCo 에서 드러남)
    pick_above = make_pose(-0.45, -0.20, 0.30)
    place_above = make_pose(-0.45, 0.20, 0.30)

    arm.move_to_joints(READY, '준비 자세')
    arm.open_gripper()

    # 집기
    arm.move_to_pose(pick_above, '집기 위치 위로')
    arm.move_linear([lowered(pick_above, APPROACH_HEIGHT)], '직선 하강')
    arm.close_gripper()
    arm.move_linear([pick_above], '직선 상승')

    # 옮기기: 같은 높이에서 옆으로 직선 이동
    arm.move_linear([place_above], '수평 직선 이동')

    # 놓기
    arm.move_linear([lowered(place_above, APPROACH_HEIGHT)], '직선 하강')
    arm.open_gripper()
    arm.move_linear([place_above], '직선 상승')

    arm.move_to_joints(READY, '준비 자세 복귀')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
