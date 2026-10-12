"""5교시 실습 2: MoveIt 2 로 UR5e 움직이기 — 4교시 코드를 그대로 쓰고 "로봇에 맞는 값"만 바꾼다.

edu_arm (4교시)          →  UR5e 공식 설정
  플래닝 그룹 arm         →  ur_manipulator
  base_link = DH 기준     →  base_link 는 DH 기준(base)과 z축 180° 차이 → 절대 좌표 대신 "현재 위치 기준 상대 이동" 사용
  속도 30%                →  실물 첫 실습은 10%

안전을 위해
  - 시작 전에 Enter 를 눌러 확인한다 (--yes 로 생략 가능, mock 테스트용)
  - 먼저 준비 자세(관절 목표)로 이동한다 — 로봇이 팔을 다 편 자세(특이점 · 작업 영역 끝)에서 시작하면
    직선 이동이 불가능하기 때문 (mock UR5e 는 팔이 위로 똑바로 선 자세에서 시작한다)
  - 준비 자세에서 5 cm 씩만 움직인다 (위 → 옆 → 원위치)
  - 펜던트의 속도 슬라이더도 낮춰 둔다 (실제 속도 = MoveIt 속도 × 슬라이더)

실행:
  ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=<로봇 IP>
  ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
  ros2 run day1_examples c5_ur_move
"""
import copy
import sys

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

import day1_examples.moveit_helper as mh
from day1_examples.common import joint_positions_by_name

STEP = 0.05   # 상대 이동 거리 (m)
# UR 에서 흔히 쓰는 준비 자세: 팔을 접고 그리퍼가 아래를 향함 (shoulder_pan 은 현재 값 유지)
READY = [None, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]


def current_tool_pose(node):
    """TF 로 base_link -> tool0 현재 자세를 읽는다."""
    buffer = Buffer()
    TransformListener(buffer, node)
    end = node.get_clock().now().nanoseconds + 5_000_000_000
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            tf = buffer.lookup_transform(mh.FRAME, mh.EE_LINK, rclpy.time.Time()).transform
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = tf.translation.x, tf.translation.y, tf.translation.z
            pose.orientation = tf.rotation
            return pose
        except Exception:
            continue
    return None


def shifted(pose, dx=0.0, dy=0.0, dz=0.0):
    p = copy.deepcopy(pose)
    p.position.x += dx
    p.position.y += dy
    p.position.z += dz
    return p


def main():
    mh.GROUP = 'ur_manipulator'      # UR 공식 MoveIt 설정의 플래닝 그룹
    rclpy.init()
    node = Node('c5_ur_move')
    arm = mh.MoveItHelper(node)
    arm.velocity_scaling = 0.1
    if not arm.wait_for_servers():
        return
    arm.clear_paths()

    node.get_logger().info('계획: 준비 자세(관절 목표) → 위로 5 cm → 옆(y)으로 5 cm → 원위치, 속도 10%')
    if '--yes' not in sys.argv:
        input('\n>>> 로봇 주변이 비어 있고, 비상 정지 버튼을 누를 준비가 됐으면 Enter <<<\n')

    state = {}
    node.create_subscription(JointState, '/joint_states',
                             lambda m: state.setdefault('q', joint_positions_by_name(m)), 10)
    while rclpy.ok() and 'q' not in state:
        rclpy.spin_once(node, timeout_sec=0.1)
    ready = [state['q'][0]] + READY[1:]
    if not arm.move_to_joints(ready, '준비 자세'):
        node.get_logger().error('준비 자세로 이동 실패 → 중단')
        return

    start = current_tool_pose(node)
    if start is None:
        node.get_logger().error('TF(base_link -> tool0)를 읽지 못했습니다. 드라이버가 실행 중인지 확인하세요.')
        return
    p = start.position
    node.get_logger().info(f'준비 자세 tool0 (base_link 기준): ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m')

    up = shifted(start, dz=STEP)
    side = shifted(up, dy=STEP)
    for target, label in [(up, '위로 (직선)'), (side, '옆으로 (직선)'), (up, '옆 복귀 (직선)'), (start, '원위치 (직선)')]:
        if not arm.move_linear([target], label):
            node.get_logger().error('직선 이동 실패 → 중단')
            break

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
