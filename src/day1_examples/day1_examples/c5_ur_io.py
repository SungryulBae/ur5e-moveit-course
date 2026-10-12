"""5교시 실습 3: ROS 2 명령을 UR5e 컨트롤러로 전달하기 — 디지털 출력 · 속도 슬라이더

  /io_and_status_controller/set_io            디지털 출력 켜기 / 끄기 (공압 그리퍼 · 램프 등이 연결되는 곳)
  /io_and_status_controller/set_speed_slider  펜던트 속도 슬라이더 값 바꾸기

실행:
  ros2 run day1_examples c5_ur_io                      # 디지털 출력 0번을 켰다 끄고, 속도 슬라이더 30% → 원래 값
  ros2 run day1_examples c5_ur_io --pin 3              # 다른 출력 핀
※ 실물에서는 해당 출력에 무엇이 연결돼 있는지 먼저 확인할 것 (그리퍼가 연결돼 있으면 실제로 열리고 닫힌다)
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64
from ur_msgs.srv import SetIO, SetSpeedSliderFraction

NS = '/io_and_status_controller'


def call(node, client, request, label):
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f'{client.srv_name} 서비스가 없습니다 (드라이버 실행 여부 확인)')
        return False
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    ok = future.result() is not None and future.result().success
    node.get_logger().info(f'{label}: ' + ('성공' if ok else '실패'))
    return ok


def main():
    rclpy.init()
    node = Node('c5_ur_io')
    pin = int(sys.argv[sys.argv.index('--pin') + 1]) if '--pin' in sys.argv else 0

    set_io = node.create_client(SetIO, f'{NS}/set_io')
    for state, word in [(SetIO.Request.STATE_ON, '켜기'), (SetIO.Request.STATE_OFF, '끄기')]:
        req = SetIO.Request(fun=SetIO.Request.FUN_SET_DIGITAL_OUT, pin=pin, state=float(state))
        call(node, set_io, req, f'디지털 출력 {pin}번 {word}')
        time.sleep(1.0)

    # 현재 슬라이더 값을 읽어 두었다가 복원
    current = {}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Float64, '/speed_scaling_state_broadcaster/speed_scaling',
                             lambda m: current.setdefault('v', m.data / 100.0), latched)   # 토픽은 % 단위
    end = time.monotonic() + 3.0
    while 'v' not in current and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    original = current.get('v', 1.0)

    slider = node.create_client(SetSpeedSliderFraction, f'{NS}/set_speed_slider')
    call(node, slider, SetSpeedSliderFraction.Request(speed_slider_fraction=0.3), '속도 슬라이더 30%')
    time.sleep(1.0)
    call(node, slider, SetSpeedSliderFraction.Request(speed_slider_fraction=float(original)),
         f'속도 슬라이더 원래 값({original * 100:.0f}%)으로 복원')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
