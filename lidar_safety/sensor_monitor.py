"""라이다 감지 모니터.

인지 노드와 별개로 라이다 원신호를 직접 보고, 믿을 수 없으면 /sensor_ok=False를 내보낸다.
정지 노드는 이 값이 False면 거리와 상관없이 전진을 막는다.

검사 항목 (정상 시뮬 /scan에서 잰 값을 기준으로 임계값을 잡았다)
  invalid : 정면 구간에서 0이나 nan 같은 무효값 비율        정상 0%      -> 30% 넘으면 이상
  stale   : 측정 시각(header.stamp)이 현재보다 얼마나 늦었나  정상 ±0.08s  -> 0.3s 넘으면 이상
  gap     : 메시지가 아예 안 들어온 시간                     정상 0.2s    -> 0.5s 넘으면 이상
  frozen  : 직전 스캔과 완전히 같은 스캔이 연속으로 몇 번    정상 0번     -> 3번이면 이상
  noisy   : 이웃 빔 간 거리 차이의 중앙값                    정상 ~0.017m -> 0.035m 넘으면 이상

시뮬레이터 시각으로 나이를 재야 하므로 use_sim_time:=true로 실행해야 한다.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')
        self.declare_parameter('scan_topic', '/scan_faulty')
        self.declare_parameter('front_half_angle_deg', 30.0)
        self.declare_parameter('max_invalid_ratio', 0.3)
        self.declare_parameter('max_age', 0.3)
        self.declare_parameter('max_gap', 0.5)
        self.declare_parameter('max_frozen', 3)
        self.declare_parameter('max_roughness', 0.035)
        self.declare_parameter('ok_count_to_recover', 3)  # 연속 이만큼 정상이어야 복구

        self.p = {name: self.get_parameter(name).value for name in (
            'front_half_angle_deg', 'max_invalid_ratio', 'max_age', 'max_gap',
            'max_frozen', 'max_roughness', 'ok_count_to_recover')}
        self.half_angle = math.radians(self.p['front_half_angle_deg'])

        self.prev_ranges = None
        self.frozen_count = 0
        self.ok_streak = 0
        self.last_rx = None
        self.ok = False  # 첫 스캔을 확인하기 전까지는 믿지 않는다
        self.reasons = ['no data yet']

        scan_topic = self.get_parameter('scan_topic').value
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.ok_pub = self.create_publisher(Bool, '/sensor_ok', 10)
        self.state_pub = self.create_publisher(String, '/monitor_state', 10)
        self.create_timer(0.1, self.on_timer)
        self.get_logger().info(f'monitoring {scan_topic}')

    def check(self, msg: LaserScan):
        reasons = []

        front = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(math.atan2(math.sin(angle), math.cos(angle))) <= self.half_angle:
                front.append(r)
        # inf는 "반사 없음(멀다)"이라 정상일 수 있다. 0이나 nan, range_min 미만은 센서가 낸 오류값이다
        invalid = sum(1 for r in front if math.isnan(r) or (not math.isinf(r) and r < msg.range_min))
        if front and invalid / len(front) > self.p['max_invalid_ratio']:
            reasons.append(f'invalid {invalid}/{len(front)}')

        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        if age > self.p['max_age']:
            reasons.append(f'stale {age:.2f}s')

        ranges = list(msg.ranges)
        self.frozen_count = self.frozen_count + 1 if ranges == self.prev_ranges else 0
        self.prev_ranges = ranges
        if self.frozen_count >= self.p['max_frozen']:
            reasons.append(f'frozen x{self.frozen_count}')

        diffs = sorted(abs(ranges[i] - ranges[i - 1]) for i in range(1, len(ranges))
                       if math.isfinite(ranges[i]) and math.isfinite(ranges[i - 1])
                       and ranges[i] >= msg.range_min and ranges[i - 1] >= msg.range_min)
        if diffs and diffs[len(diffs) // 2] > self.p['max_roughness']:
            reasons.append(f'noisy {diffs[len(diffs) // 2]:.3f}m')

        return reasons

    def on_scan(self, msg: LaserScan):
        self.last_rx = self.get_clock().now()
        reasons = self.check(msg)
        if reasons:
            self.ok_streak = 0
            self.set_state(False, reasons)
        else:
            self.ok_streak += 1
            if self.ok or self.ok_streak >= self.p['ok_count_to_recover']:
                self.set_state(True, [])

    def on_timer(self):
        # 메시지가 끊기면 on_scan이 안 불리므로 타이머에서 따로 본다
        if self.last_rx is not None:
            gap = (self.get_clock().now() - self.last_rx).nanoseconds / 1e9
            if gap > self.p['max_gap']:
                self.ok_streak = 0
                self.set_state(False, [f'gap {gap:.1f}s'])
        self.ok_pub.publish(Bool(data=self.ok))
        self.state_pub.publish(String(data='OK' if self.ok else ', '.join(self.reasons)))

    def set_state(self, ok, reasons):
        if ok != self.ok:
            if ok:
                self.get_logger().info('sensor OK')
            else:
                self.get_logger().warn(f'SENSOR FAULT: {", ".join(reasons)}')
        self.ok = ok
        self.reasons = reasons


def main():
    rclpy.init()
    node = SensorMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
