"""5교시 실습 1: UR5e 상태 읽기 — 티칭펜던트에 보이는 정보를 ROS 2 토픽으로 받는다.

  /io_and_status_controller/robot_mode            로봇 모드 (전원 꺼짐 · 대기 · 동작 중 …)
  /io_and_status_controller/safety_mode           안전 모드 (정상 · 보호 정지 · 비상 정지 …)
  /io_and_status_controller/robot_program_running 펜던트에서 External Control 프로그램이 실행 중인가
  /speed_scaling_state_broadcaster/speed_scaling  펜던트 속도 슬라이더 값 (0 ~ 100 %)
  /io_and_status_controller/io_states             디지털 입출력 상태
  TF base -> tool0                                 펜던트의 TCP 좌표와 같은 기준(base)의 끝단 위치

실행 (실물 / mock 공통):
  ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=<로봇 IP>
  ros2 run day1_examples c5_ur_status
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64
from tf2_ros import Buffer, TransformListener
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_msgs.msg import IOStates

ROBOT_MODES = {-1: '컨트롤러 없음', 0: '연결 끊김', 1: '안전 확인 필요', 2: '부팅 중', 3: '전원 꺼짐',
               4: '전원 켜짐', 5: '대기(브레이크 잠김)', 6: '백드라이브', 7: '동작 가능(RUNNING)',
               8: '펌웨어 업데이트'}
SAFETY_MODES = {1: '정상', 2: '축소 모드', 3: '보호 정지', 4: '복구 모드', 5: '안전 장치 정지',
                6: '시스템 비상 정지', 7: '로봇 비상 정지', 8: '위반', 9: '고장',
                10: '안전 장치 검증 필요', 11: '안전 장치 비상 정지'}


class URStatus(Node):
    def __init__(self):
        super().__init__('ur_status')
        self.s = {}
        ns = '/io_and_status_controller'
        # UR 상태 토픽은 "값이 바뀔 때만" 보내고 마지막 값을 저장해 둔다(transient_local) → 같은 QoS 로 받아야 한다
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(RobotMode, f'{ns}/robot_mode', lambda m: self.s.update(robot=m.mode), latched)
        self.create_subscription(SafetyMode, f'{ns}/safety_mode', lambda m: self.s.update(safety=m.mode), latched)
        self.create_subscription(Bool, f'{ns}/robot_program_running',
                                 lambda m: self.s.update(program=m.data), latched)
        self.create_subscription(Float64, '/speed_scaling_state_broadcaster/speed_scaling',
                                 lambda m: self.s.update(speed=m.data), latched)
        self.create_subscription(IOStates, f'{ns}/io_states',
                                 lambda m: self.s.update(dout=[int(p.state) for p in m.digital_out_states[:8]]),
                                 latched)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(1.0, self.report)

    def report(self):
        s = self.s
        tcp = '(TF 대기 중)'
        try:
            t = self.buffer.lookup_transform('base', 'tool0', rclpy.time.Time()).transform.translation
            tcp = f'({t.x * 1000:.1f}, {t.y * 1000:.1f}, {t.z * 1000:.1f}) mm'
        except Exception:
            pass
        robot = ROBOT_MODES.get(s.get('robot'), '?')
        safety = SAFETY_MODES.get(s.get('safety'), '?')
        program = {True: '실행 중', False: '정지'}.get(s.get('program'), '?')
        speed = f"{s['speed']:.0f}%" if 'speed' in s else '?'   # 0 ~ 100 (%) 단위로 들어온다
        self.get_logger().info(
            f'로봇: {robot} | 안전: {safety} | External Control: {program} | 속도 슬라이더: {speed}\n'
            f'    디지털 출력 0~7: {s.get("dout", "?")} | TCP(base 기준): {tcp}')


def main():
    rclpy.init()
    node = URStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
