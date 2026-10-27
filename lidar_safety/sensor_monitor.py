"""라이다 감지 모니터.

인지 노드와 별개로 라이다 원신호를 직접 보고 센서 상태를 세 단계로 내보낸다 (/sensor_level).
  0 OK       : 정상
  1 DEGRADED : 거리값은 쓸 만하지만 품질이 떨어짐 -> 정지 노드가 감속
  2 FAULT    : 믿을 수 없음 -> 정지 노드가 전진을 막음
degrade_on_noise가 꺼져 있으면 DEGRADED 없이 모든 이상을 FAULT로 본다.
/sensor_ok는 FAULT가 아니면 True다.

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
from std_msgs.msg import Bool, String, UInt8

OK, DEGRADED, FAULT = 0, 1, 2
LEVEL_NAMES = {OK: 'OK', DEGRADED: 'DEGRADED', FAULT: 'FAULT'}
# 노이즈는 최소 거리를 작게(보수적으로) 만들 뿐 장애물을 숨기지는 않아서, 감속하며 계속 쓸 수 있다고 본다
SOFT_CHECKS = ('noisy',)


class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')
        self.declare_parameter('scan_topic', '/scan_faulty')
        self.declare_parameter('front_half_angle_deg', 30.0)
        self.declare_parameter('max_invalid_ratio', 0.3)
        self.declare_parameter('max_age', 0.3)
        self.declare_parameter('max_gap', 0.5)
        self.declare_parameter('max_frozen', 3)
        self.declare_parameter('min_finite_beams', 10)  # frozen 검사를 하려면 측정값이 이만큼은 있어야 함
        self.declare_parameter('max_roughness', 0.035)
        self.declare_parameter('ok_count_to_recover', 3)  # 연속 이만큼 정상이어야 복구
        self.declare_parameter('degrade_on_noise', False)
        self.degrade_on_noise = self.get_parameter('degrade_on_noise').value

        self.p = {name: self.get_parameter(name).value for name in (
            'front_half_angle_deg', 'max_invalid_ratio', 'max_age', 'max_gap',
            'max_frozen', 'min_finite_beams', 'max_roughness', 'ok_count_to_recover')}
        self.half_angle = math.radians(self.p['front_half_angle_deg'])

        self.prev_ranges = None
        self.frozen_count = 0
        self.better_streak = 0  # 지금 단계보다 좋은 스캔이 연속으로 몇 번 왔나
        self.better_worst = OK  # 그 연속 구간에서 가장 나빴던 단계
        self.last_rx = None
        self.level = FAULT  # 첫 스캔을 확인하기 전까지는 믿지 않는다
        self.reasons = ['no data yet']

        scan_topic = self.get_parameter('scan_topic').value
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.ok_pub = self.create_publisher(Bool, '/sensor_ok', 10)
        self.level_pub = self.create_publisher(UInt8, '/sensor_level', 10)
        self.state_pub = self.create_publisher(String, '/monitor_state', 10)
        self.create_timer(0.1, self.on_timer)
        self.get_logger().info(f'monitoring {scan_topic}, degrade_on_noise={self.degrade_on_noise}')

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
        # 측정 범위 안의 값이 거의 없으면(사방이 멀어 전부 inf 등) 노이즈가 안 섞여 정상이어도 스캔이 똑같다.
        # 이럴 때는 멈춤 여부를 판단할 근거가 없으므로 세지 않는다
        finite = sum(1 for r in ranges if math.isfinite(r) and r >= msg.range_min)
        if finite >= self.p['min_finite_beams'] and ranges == self.prev_ranges:
            self.frozen_count += 1
        else:
            self.frozen_count = 0
        self.prev_ranges = ranges
        if self.frozen_count >= self.p['max_frozen']:
            reasons.append(f'frozen x{self.frozen_count}')

        diffs = sorted(abs(ranges[i] - ranges[i - 1]) for i in range(1, len(ranges))
                       if math.isfinite(ranges[i]) and math.isfinite(ranges[i - 1])
                       and ranges[i] >= msg.range_min and ranges[i - 1] >= msg.range_min)
        if diffs and diffs[len(diffs) // 2] > self.p['max_roughness']:
            reasons.append(f'noisy {diffs[len(diffs) // 2]:.3f}m')

        return reasons

    def classify(self, reasons):
        if not reasons:
            return OK
        if self.degrade_on_noise and all(r.split(' ')[0] in SOFT_CHECKS for r in reasons):
            return DEGRADED
        return FAULT

    def on_scan(self, msg: LaserScan):
        self.last_rx = self.get_clock().now()
        reasons = self.check(msg)
        level = self.classify(reasons)
        if level >= self.level:
            # 나빠지는 쪽은 바로 반영한다
            self.better_streak = 0
            self.set_level(level, reasons)
            return
        # 좋아지는 쪽은 연속으로 확인된 뒤에만 반영한다
        self.better_worst = level if self.better_streak == 0 else max(self.better_worst, level)
        self.better_streak += 1
        if self.better_streak >= self.p['ok_count_to_recover']:
            self.better_streak = 0
            self.set_level(self.better_worst, reasons)

    def on_timer(self):
        # 메시지가 끊기면 on_scan이 안 불리므로 타이머에서 따로 본다
        if self.last_rx is not None:
            gap = (self.get_clock().now() - self.last_rx).nanoseconds / 1e9
            if gap > self.p['max_gap']:
                self.better_streak = 0
                self.set_level(FAULT, [f'gap {gap:.1f}s'])
        self.ok_pub.publish(Bool(data=self.level != FAULT))
        self.level_pub.publish(UInt8(data=self.level))
        state = 'OK' if self.level == OK else ', '.join(self.reasons)
        if self.level == DEGRADED:
            state = f'DEGRADED {state}'
        self.state_pub.publish(String(data=state))

    def set_level(self, level, reasons):
        if level != self.level:
            text = ', '.join(reasons)
            if level == OK:
                self.get_logger().info('sensor OK')
            elif level == DEGRADED:
                self.get_logger().warn(f'SENSOR DEGRADED: {text}')
            else:
                self.get_logger().warn(f'SENSOR FAULT: {text}')
        self.level = level
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
