"""라이다 결함 주입 노드.

/scan을 받아 결함을 섞은 뒤 /scan_faulty로 내보낸다.
fault 파라미터는 실행 중에도 바꿀 수 있다.
  ros2 param set /fault_injector fault zero

결함 종류
  none   : 그대로 통과
  zero   : zero_ratio 비율의 빔을 0.0으로 (실센서에서 반사가 안 돌아올 때와 비슷)
  noise  : 유효한 값에 표준편차 noise_std [m]의 가우시안 노이즈
  delay  : delay_sec 초 늦게 내보냄 (header.stamp는 원래 측정 시각 그대로)
  freeze : 결함이 시작된 순간의 값을 계속 반복 (stamp만 새로 찍힘)
"""
import collections
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

FAULTS = ('none', 'zero', 'noise', 'delay', 'freeze')


class FaultInjector(Node):
    def __init__(self):
        super().__init__('fault_injector')
        self.declare_parameter('fault', 'none')
        self.declare_parameter('zero_ratio', 1.0)
        self.declare_parameter('noise_std', 0.05)
        self.declare_parameter('delay_sec', 0.5)
        self.declare_parameter('seed', 0)

        random.seed(self.get_parameter('seed').value)
        self.delay_queue = collections.deque()  # (내보낼 시각, msg)
        self.frozen = None
        self.prev_fault = None

        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_faulty', 10)
        # 평가할 때 "언제 어떤 결함이 들어갔는지"를 정답으로 쓰기 위해 함께 내보낸다
        self.state_pub = self.create_publisher(String, '/fault_state', 10)
        self.create_timer(0.01, self.flush_delayed)

    def param(self, name):
        return self.get_parameter(name).value

    def on_scan(self, msg: LaserScan):
        fault = self.param('fault')
        if fault not in FAULTS:
            self.get_logger().error(f'unknown fault "{fault}", use one of {FAULTS}',
                                    throttle_duration_sec=2.0)
            fault = 'none'
        if fault != self.prev_fault:
            self.get_logger().warn(f'fault: {self.prev_fault} -> {fault}')
            self.frozen = None
            self.prev_fault = fault
        self.state_pub.publish(String(data=fault))

        if fault == 'zero':
            ratio = self.param('zero_ratio')
            msg.ranges = [0.0 if random.random() < ratio else r for r in msg.ranges]
        elif fault == 'noise':
            std = self.param('noise_std')
            msg.ranges = [r + random.gauss(0.0, std) if r != float('inf') else r for r in msg.ranges]
        elif fault == 'freeze':
            if self.frozen is None:
                self.frozen = list(msg.ranges)
            msg.ranges = self.frozen
        elif fault == 'delay':
            release = self.get_clock().now().nanoseconds + int(self.param('delay_sec') * 1e9)
            self.delay_queue.append((release, msg))
            return

        self.scan_pub.publish(msg)

    def flush_delayed(self):
        now = self.get_clock().now().nanoseconds
        while self.delay_queue and self.delay_queue[0][0] <= now:
            self.scan_pub.publish(self.delay_queue.popleft()[1])
        # delay를 끄면 밀린 메시지는 버린다
        if self.prev_fault != 'delay':
            self.delay_queue.clear()


def main():
    rclpy.init()
    node = FaultInjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
