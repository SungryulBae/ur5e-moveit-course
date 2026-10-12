"""3교시 실습 2: 자코비안 기반 수치 IK를 직접 구현하고, 결과를 로봇에 보내 확인한다.

알고리즘 (감쇠 최소제곱, damped least squares):
  1. 현재 관절값 q 에서 FK -> 현재 끝단 자세 T
  2. 오차 e = [위치 오차; 자세 오차(축-각)]  (6x1)
  3. 자코비안 J(q) (6x6) 로 dq = J^T (J J^T + λ²I)^-1 e
  4. q <- q + dq, 오차가 충분히 작아질 때까지 반복

실행:
  ros2 launch edu_arm_description control.launch.py
  ros2 run day1_examples c3_ik_numeric                    # 기본 목표
  ros2 run day1_examples c3_ik_numeric 0.4 0.2 0.3         # 목표 위치 x y z (m), 그리퍼는 아래 방향
"""
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from day1_examples.c2_joint_commander import JointCommander
from day1_examples.common import (DH_A, DH_ALPHA, DH_D, dh_transform, fk,
                                  joint_positions_by_name, rpy_to_rot)


def jacobian(q):
    """기하학적 자코비안 (base_link 기준). i번째 열 = [z_{i-1} x (p_e - p_{i-1}); z_{i-1}]."""
    T = np.eye(4)
    origins, axes = [T[:3, 3].copy()], [T[:3, 2].copy()]
    for i in range(6):
        T = T @ dh_transform(q[i], DH_D[i], DH_A[i], DH_ALPHA[i])
        origins.append(T[:3, 3].copy())
        axes.append(T[:3, 2].copy())
    p_end = origins[-1]
    J = np.zeros((6, 6))
    for i in range(6):
        J[:3, i] = np.cross(axes[i], p_end - origins[i])
        J[3:, i] = axes[i]
    return J


def rotation_error(R_current, R_target):
    """두 회전의 차이를 축 * 각 벡터로 (base 좌표계 기준)."""
    R_err = R_target @ R_current.T
    angle = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1.0, 1.0))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([R_err[2, 1] - R_err[1, 2],
                     R_err[0, 2] - R_err[2, 0],
                     R_err[1, 0] - R_err[0, 1]]) / (2 * np.sin(angle))
    return axis * angle


def solve_ik(T_target, q_init, max_iter=200, damping=0.05, tol_pos=1e-4, tol_rot=1e-3):
    q = np.array(q_init, dtype=float)
    for it in range(max_iter):
        T = fk(q)
        e = np.concatenate([T_target[:3, 3] - T[:3, 3],
                            rotation_error(T[:3, :3], T_target[:3, :3])])
        if np.linalg.norm(e[:3]) < tol_pos and np.linalg.norm(e[3:]) < tol_rot:
            return q, it, True
        J = jacobian(q)
        dq = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(6), e)
        q = q + dq
    return q, max_iter, False


def main():
    rclpy.init()
    node = Node('ik_numeric')

    # 현재 관절값을 초기값(시드)으로 사용
    state = {}
    node.create_subscription(JointState, '/joint_states',
                             lambda m: state.setdefault('q', joint_positions_by_name(m)), 10)
    while rclpy.ok() and 'q' not in state:
        rclpy.spin_once(node, timeout_sec=0.1)
    q0 = state['q']

    xyz = [float(v) for v in sys.argv[1:4]] if len(sys.argv) >= 4 else [-0.4, -0.2, 0.3]
    T_target = np.eye(4)
    T_target[:3, :3] = rpy_to_rot(np.pi, 0.0, 0.0)  # 그리퍼가 바닥(-z)을 향하도록
    T_target[:3, 3] = xyz

    q, iters, ok = solve_ik(T_target, q0)
    np.set_printoptions(precision=4, suppress=True)
    node.get_logger().info(f'목표 위치: {np.array(xyz)}')
    if not ok:
        node.get_logger().error(f'IK 수렴 실패 ({iters}회). 작업 영역 밖이거나 특이점 근처일 수 있습니다.')
        rclpy.try_shutdown()
        return
    node.get_logger().info(f'IK 해 ({iters}회 반복): {np.degrees(q).round(2)} 도')
    node.get_logger().info(f'검산 FK(해): {fk(q)[:3, 3]}')
    node.destroy_node()

    commander = JointCommander()
    commander.client.wait_for_server(timeout_sec=10.0)
    commander.move(q)
    commander.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
