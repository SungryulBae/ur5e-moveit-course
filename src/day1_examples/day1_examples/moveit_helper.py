"""4교시 공통: MoveIt 2 move_group 에 서비스 · 액션으로 요청하는 작은 헬퍼.

MoveIt이 실제로 받는 메시지가 무엇인지 보이도록 일부러 래퍼 라이브러리 없이 직접 채운다.
  - /move_action          (MoveGroup 액션)       : 목표 -> IK -> 경로 계획 -> 실행
  - /compute_cartesian_path (GetCartesianPath 서비스): 직선 경로 계획
  - /execute_trajectory   (ExecuteTrajectory 액션): 계획된 궤적 실행
  - /apply_planning_scene (ApplyPlanningScene 서비스): 충돌 물체 추가 · 제거
  - /gripper_controller/gripper_cmd (GripperCommand 액션): 그리퍼
  - /compute_fk           (GetPositionFK 서비스): 계획된 궤적의 끝단 경로 계산 → RViz 에 선으로 표시
"""
import numpy as np
import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint,
                             MotionPlanRequest, MoveItErrorCodes,
                             OrientationConstraint, PlanningScene,
                             PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionFK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from day1_examples.common import ARM_JOINTS, rot_to_quat, rpy_to_rot

GROUP = 'arm'
FRAME = 'base_link'
EE_LINK = 'tool0'

GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.04

# RViz 끝단 경로 색: 포즈 · 관절 목표(관절 공간 계획) = 주황, 직선 이동(카테시안) = 초록
COLOR_JOINT_SPACE = ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0)
COLOR_CARTESIAN = ColorRGBA(r=0.1, g=0.9, b=0.2, a=1.0)


def error_name(code):
    """MoveItErrorCodes 숫자 -> 이름 (예: -31 -> NO_IK_SOLUTION)."""
    for name in dir(MoveItErrorCodes):
        if name.isupper() and getattr(MoveItErrorCodes, name) == code:
            return f'{name} ({code})'
    return str(code)


def make_pose(x, y, z, roll=np.pi, pitch=0.0, yaw=0.0):
    """위치(m) + RPY(rad) -> geometry_msgs/Pose. 기본 자세는 그리퍼가 바닥을 향함."""
    q = rot_to_quat(rpy_to_rot(roll, pitch, yaw))
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
    return pose


class MoveItHelper:
    def __init__(self, node: Node):
        self.node = node
        self.log = node.get_logger()
        self.move_client = ActionClient(node, MoveGroup, '/move_action')
        self.exec_client = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        self.gripper_client = ActionClient(node, GripperCommand, '/gripper_controller/gripper_cmd')
        self.cartesian_srv = node.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.scene_srv = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.fk_srv = node.create_client(GetPositionFK, '/compute_fk')
        self.marker_pub = node.create_publisher(MarkerArray, '/day1_markers', 10)
        self._marker_id = 0
        self.velocity_scaling = 0.3
        self.planner_id = ''          # '' = 기본(OMPL RRTConnect)
        self.pipeline_id = ''         # 'pilz_industrial_motion_planner' 로 바꾸면 PTP/LIN 사용 가능

    # ---------------- 공통 ----------------
    def wait_for_servers(self, timeout=15.0):
        ok = self.move_client.wait_for_server(timeout_sec=timeout)
        ok &= self.exec_client.wait_for_server(timeout_sec=timeout)
        ok &= self.cartesian_srv.wait_for_service(timeout_sec=timeout)
        if not ok:
            self.log.error('move_group 이 없습니다. edu_arm_moveit_config demo.launch.py 를 먼저 실행하세요.')
        return ok

    def _spin(self, future):
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    def _send_action(self, client, goal):
        handle = self._spin(client.send_goal_async(goal))
        if not handle.accepted:
            self.log.error('목표가 거부되었습니다')
            return None
        return self._spin(handle.get_result_async()).result

    # ---------------- 목표 -> 계획 -> 실행 ----------------
    def _plan_and_execute(self, constraints, label):
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.pipeline_id = self.pipeline_id
        req.planner_id = self.planner_id
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.velocity_scaling
        req.max_acceleration_scaling_factor = self.velocity_scaling
        req.goal_constraints = [constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False     # 계획 후 바로 실행
        goal.planning_options.replan = True

        self.log.info(f'[{label}] 계획 + 실행 요청')
        result = self._send_action(self.move_client, goal)
        ok = result is not None and result.error_code.val == MoveItErrorCodes.SUCCESS
        if ok:
            n = len(result.planned_trajectory.joint_trajectory.points)
            self.log.info(f'[{label}] 성공 (궤적 점 {n}개)')
            self.draw_path(result.planned_trajectory, COLOR_JOINT_SPACE, label)
        else:
            code = None if result is None else result.error_code.val
            self.log.error(f'[{label}] 실패 ({error_name(code)})')
        return ok

    def move_to_joints(self, q, label='관절 목표'):
        c = Constraints()
        for name, value in zip(ARM_JOINTS, q):
            jc = JointConstraint(joint_name=name, position=float(value),
                                 tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
            c.joint_constraints.append(jc)
        return self._plan_and_execute(c, label)

    def move_to_pose(self, pose: Pose, label='포즈 목표'):
        """끝단(tool0) 포즈 목표: 위치 구속(작은 구) + 자세 구속 -> MoveIt 이 IK 를 풀고 경로를 계획한다."""
        pc = PositionConstraint()
        pc.header.frame_id = FRAME
        pc.link_name = EE_LINK
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
        pc.constraint_region.primitives = [sphere]
        pc.constraint_region.primitive_poses = [Pose(position=pose.position)]
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = FRAME
        oc.link_name = EE_LINK
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0

        c = Constraints(position_constraints=[pc], orientation_constraints=[oc])
        return self._plan_and_execute(c, label)

    # ---------------- 직선 운동 ----------------
    def move_linear(self, waypoints, label='직선 이동', max_step=0.005):
        """끝단을 waypoint 들을 잇는 직선으로 움직인다 (GetCartesianPath -> ExecuteTrajectory)."""
        req = GetCartesianPath.Request()
        req.header.frame_id = FRAME
        req.group_name = GROUP
        req.link_name = EE_LINK
        req.waypoints = waypoints
        req.max_step = max_step
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = self.velocity_scaling
        req.max_acceleration_scaling_factor = self.velocity_scaling
        res = self._spin(self.cartesian_srv.call_async(req))
        self.log.info(f'[{label}] 직선 경로 달성률 fraction = {res.fraction * 100:.1f}%')
        if res.fraction < 0.99:
            self.log.error(f'[{label}] 경로 일부만 계획됨 → 실행하지 않음')
            return False
        goal = ExecuteTrajectory.Goal(trajectory=res.solution)
        result = self._send_action(self.exec_client, goal)
        ok = result is not None and result.error_code.val == MoveItErrorCodes.SUCCESS
        self.log.info(f'[{label}] ' + ('완료' if ok else '실행 실패'))
        if ok:
            self.draw_path(res.solution, COLOR_CARTESIAN, label)
        return ok

    # ---------------- 끝단 경로 시각화 ----------------
    def ee_path(self, robot_trajectory):
        """궤적의 각 점(관절값)을 /compute_fk 로 끝단(tool0) 위치로 바꾼다."""
        jt = robot_trajectory.joint_trajectory
        points = []
        for pt in jt.points:
            req = GetPositionFK.Request()
            req.header.frame_id = FRAME
            req.fk_link_names = [EE_LINK]
            req.robot_state.joint_state = JointState(name=jt.joint_names, position=list(pt.positions))
            res = self._spin(self.fk_srv.call_async(req))
            if res.error_code.val == MoveItErrorCodes.SUCCESS and res.pose_stamped:
                p = res.pose_stamped[0].pose.position
                points.append([p.x, p.y, p.z])
        return np.array(points)

    def draw_path(self, robot_trajectory, color, label):
        """끝단이 지나간 경로를 RViz(/day1_markers)에 선으로 그리고, 직선에서 얼마나 벗어났는지 출력한다."""
        if not self.fk_srv.wait_for_service(timeout_sec=2.0):
            return
        path = self.ee_path(robot_trajectory)
        if len(path) < 2 or np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)) < 0.001:
            return   # 이미 목표에 있어 움직이지 않은 경우
        start, end = path[0], path[-1]
        chord = end - start
        length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
        if np.linalg.norm(chord) > 1e-6:
            u = chord / np.linalg.norm(chord)
            rel = path - start
            deviation = np.max(np.linalg.norm(rel - np.outer(rel @ u, u), axis=1))
            self.log.info(f'[{label}] 끝단 경로: 이동 거리 {length * 1000:.0f} mm '
                          f'(직선 거리 {np.linalg.norm(chord) * 1000:.0f} mm), '
                          f'직선에서 최대 {deviation * 1000:.1f} mm 벗어남')

        line = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, ns='ee_path', id=self._next_id())
        line.header.frame_id = FRAME
        line.pose.orientation.w = 1.0
        line.scale.x = 0.006
        line.color = color
        line.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in path]

        text = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, ns='ee_path_label', id=self._next_id())
        text.header.frame_id = FRAME
        text.pose.position = Point(x=float(end[0]), y=float(end[1]), z=float(end[2]) + 0.05)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.035
        text.color = color
        text.text = label
        self.marker_pub.publish(MarkerArray(markers=[line, text]))

    def clear_paths(self):
        """RViz 에 그린 끝단 경로를 모두 지운다."""
        self.marker_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def _next_id(self):
        self._marker_id += 1
        return self._marker_id

    # ---------------- 그리퍼 ----------------
    def gripper(self, position, max_effort=20.0):
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.log.error('그리퍼 액션 서버가 없습니다')
            return False
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        result = self._send_action(self.gripper_client, goal)
        ok = result is not None and (result.reached_goal or result.stalled)
        self.log.info(f'그리퍼 -> {position:.3f} m ' + ('완료' if ok else '실패'))
        return ok

    def open_gripper(self):
        return self.gripper(GRIPPER_OPEN)

    def close_gripper(self):
        return self.gripper(GRIPPER_CLOSED)

    # ---------------- Planning Scene ----------------
    def add_box(self, name, size, xyz):
        obj = CollisionObject()
        obj.header.frame_id = FRAME
        obj.id = name
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, size)))]
        p = Pose()
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.w = 1.0
        obj.primitive_poses = [p]
        obj.operation = CollisionObject.ADD
        return self._apply_scene(obj, f'충돌 물체 추가: {name}')

    def remove_object(self, name):
        obj = CollisionObject(id=name, operation=CollisionObject.REMOVE)
        obj.header.frame_id = FRAME
        return self._apply_scene(obj, f'충돌 물체 제거: {name}', fail_text=' (없음)')

    def _apply_scene(self, obj, label, fail_text=' 실패'):
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [obj]
        res = self._spin(self.scene_srv.call_async(ApplyPlanningScene.Request(scene=scene)))
        self.log.info(label + (' 완료' if res.success else fail_text))
        return res.success
