"""라이다 장애물 인지 노드.

/scan에서 정면 구간의 최소 거리를 구해 /front_distance로 내보낸다.
나중에 결함 주입 노드를 끼우면 scan_topic만 /scan_faulty로 바꿔서 쓴다.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('front_half_angle_deg', 30.0)

        scan_topic = self.get_parameter('scan_topic').value
        self.half_angle = math.radians(self.get_parameter('front_half_angle_deg').value)

        self.sub = self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float32, '/front_distance', 10)
        self.get_logger().info(f'listening on {scan_topic}, front ±{math.degrees(self.half_angle):.0f}°')

    def on_scan(self, msg: LaserScan):
        front = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))  # [-pi, pi]로 정규화, 0이 정면
            if abs(angle) > self.half_angle:
                continue
            # 측정 범위를 벗어난 값(inf, nan, 0 등)은 "장애물 없음"이 아니라 "모름"이라 버린다
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                front.append(r)

        dist = min(front) if front else math.inf
        self.pub.publish(Float32(data=dist))
        self.get_logger().info(f'front min: {dist:.2f} m  (valid {len(front)} beams)',
                               throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
