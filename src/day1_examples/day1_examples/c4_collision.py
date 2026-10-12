"""4교시 실습 3: 충돌 회피 — Planning Scene 에 장애물을 넣으면 MoveIt 이 돌아가는 경로를 찾는다.

1. 장애물 없이 A -> B 이동 (거의 곧장 감)
2. A 와 B 사이에 벽을 세움
3. 같은 A -> B 이동 (벽을 피해 돌아감)  → RViz 에서 경로 차이를 비교
4. 직선 이동은 벽을 피할 수 없음 → fraction 이 떨어져 실패하는 것을 확인

실행:
  ros2 launch edu_arm_moveit_config demo.launch.py
  ros2 run day1_examples c4_collision
"""
import rclpy
from rclpy.node import Node

from day1_examples.common import READY
from day1_examples.moveit_helper import MoveItHelper, make_pose


def main():
    rclpy.init()
    node = Node('c4_collision')
    arm = MoveItHelper(node)
    if not arm.wait_for_servers():
        return
    arm.clear_paths()   # RViz 에 남아 있던 이전 끝단 경로 지우기 (주황 = 관절 공간 계획, 초록 = 직선)

    A = make_pose(-0.45, -0.25, 0.20)
    B = make_pose(-0.45, 0.25, 0.20)

    arm.remove_object('wall')
    arm.move_to_joints(READY, '준비 자세')
    arm.move_to_pose(A, 'A 로 이동')
    arm.move_to_pose(B, '장애물 없이 A -> B')
    arm.move_to_pose(A, 'B -> A 복귀')

    # 바닥(z=0)에서 0.45m 높이의 얇은 벽을 A 와 B 사이에 세운다
    arm.add_box('wall', size=[0.30, 0.02, 0.45], xyz=[-0.45, 0.0, 0.225])
    arm.move_to_pose(B, '벽이 있을 때 A -> B (돌아감)')
    arm.move_to_pose(A, 'B -> A 복귀')
    arm.move_linear([B], '벽을 뚫는 직선 이동 (실패 예상)')

    arm.remove_object('wall')
    arm.move_to_joints(READY, '준비 자세 복귀')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
