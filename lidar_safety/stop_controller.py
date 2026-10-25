"""정지 판단 노드.

teleop 명령(/cmd_vel_teleop)을 받아 그대로 /cmd_vel로 넘기다가,
정면 거리가 stop_distance보다 가까우면 전진 속도만 0으로 바꾼다.
회전과 후진은 막지 않아서 장애물에서 빠져나올 수 있다.
use_monitor가 켜져 있으면 감지 모니터의 /sensor_ok가 False일 때도 전진을 막는다.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class StopController(Node):
    def __init__(self):
        super().__init__('stop_controller')
        self.declare_parameter('stop_distance', 0.35)   # 이보다 가까우면 정지 [m]
        self.declare_parameter('resume_margin', 0.05)   # 다시 출발하려면 이만큼 더 멀어져야 함 [m]
        self.declare_parameter('stale_timeout', 0.5)    # 거리값이 이 시간 넘게 안 오면 정지 [s], 0이면 끔
        self.declare_parameter('use_monitor', False)    # 감지 모니터 판단으로 덮어쓸지

        self.stop_distance = self.get_parameter('stop_distance').value
        self.resume_margin = self.get_parameter('resume_margin').value
        self.stale_timeout = self.get_parameter('stale_timeout').value
        self.use_monitor = self.get_parameter('use_monitor').value

        self.cmd_in = Twist()
        self.distance = math.inf
        self.last_distance_time = None
        self.sensor_ok = False
        self.last_monitor_time = None
        self.stopped = False

        self.create_subscription(Twist, '/cmd_vel_teleop', self.on_cmd, 10)
        self.create_subscription(Float32, '/front_distance', self.on_distance, 10)
        if self.use_monitor:
            self.create_subscription(Bool, '/sensor_ok', self.on_sensor_ok, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(Bool, '/stop_state', 10)

        # teleop은 키를 누를 때만 명령을 보내므로, 거리 변화에 반응하려면 주기적으로 내보내야 한다
        self.create_timer(0.1, self.on_timer)
        self.get_logger().info(f'stop_distance={self.stop_distance} m, stale_timeout={self.stale_timeout} s, '
                               f'use_monitor={self.use_monitor}')

    def on_cmd(self, msg: Twist):
        self.cmd_in = msg

    def on_distance(self, msg: Float32):
        self.distance = msg.data
        self.last_distance_time = self.get_clock().now()

    def on_sensor_ok(self, msg: Bool):
        self.sensor_ok = msg.data
        self.last_monitor_time = self.get_clock().now()

    def is_old(self, stamp):
        if self.stale_timeout <= 0:
            return False
        if stamp is None:
            return True
        return (self.get_clock().now() - stamp).nanoseconds / 1e9 > self.stale_timeout

    def stop_reason(self):
        if self.is_old(self.last_distance_time):
            return 'stale distance'
        if self.use_monitor:
            # 모니터가 죽어서 소식이 끊겨도 정상으로 보지 않는다
            if self.is_old(self.last_monitor_time):
                return 'monitor silent'
            if not self.sensor_ok:
                return 'sensor fault'
        if self.distance < self.stop_distance:
            return f'obstacle {self.distance:.2f} m'
        return None

    def on_timer(self):
        reason = self.stop_reason()
        if reason:
            if not self.stopped:
                self.get_logger().warn(f'STOP: {reason}')
            self.stopped = True
        elif self.stopped and self.distance > self.stop_distance + self.resume_margin:
            self.get_logger().info(f'RESUME: {self.distance:.2f} m')
            self.stopped = False

        cmd = Twist()
        cmd.linear.x = self.cmd_in.linear.x
        cmd.angular.z = self.cmd_in.angular.z
        if self.stopped and cmd.linear.x > 0.0:
            cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(Bool(data=self.stopped))


def main():
    rclpy.init()
    node = StopController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())  # 꺼질 때 로봇을 세워 둔다
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
