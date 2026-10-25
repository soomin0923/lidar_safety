"""결함 주입 평가 자동화.

Gazebo에 벽을 세우고, 같은 조건으로 전진 -> 주행 중 결함 주입 -> 정지/충돌 여부를 반복 측정한다.
정답 거리는 라이다가 아니라 시뮬레이터가 아는 로봇 위치(/odom, WORLD 기준)와 벽 위치로 계산한다.

  ros2 run lidar_safety evaluate                         # 기본: 결함 5종 x 속도 2개 x 3회 x 모니터 끔/켬
  ros2 run lidar_safety evaluate --repeats 1 --faults none,zero --speeds 0.15

Gazebo는 빈 월드로 켜 두고 실행한다. 기둥이 있는 turtlebot3_world에서는 옆 기둥에 걸려 결과가 오염된다.
  ros2 launch turtlebot3_gazebo empty_world.launch.py
safety.launch.py와 teleop은 이 스크립트가 직접 다루므로 꺼 둘 것.
결과: ~/ros2_ws/eval_results/<시각>/trials.csv, summary.md, clearance.png
"""
import argparse
import csv
import datetime
import math
import os
import pathlib
import signal
import subprocess
import time

import rclpy
from gazebo_msgs.msg import PerformanceMetrics
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from lidar_safety import report

# turtlebot3_burger model.sdf 기준: 몸체 충돌 박스 0.14 x 0.14, 중심이 base에서 x=-0.032
ROBOT_FRONT_X = -0.032 + 0.07
ROBOT_HALF_WIDTH = 0.07
WALL_NAME = 'eval_wall'
WALL_THICKNESS = 0.05
WALL_WIDTH = 2.0
COLLISION_CLEARANCE = 0.01   # 이보다 가까우면 벽에 닿은 것으로 본다 [m]
FAULT_PARAMS = {'delay': {'delay_sec': 0.8}, 'noise': {'noise_std': 0.05}, 'zero': {'zero_ratio': 1.0}}
CSV_FIELDS = ['monitor', 'fault', 'speed', 'repeat', 'start_clearance', 'inject_clearance',
              'detect_latency_s', 'stop_latency_s', 'stop_clearance', 'final_clearance', 'outcome', 'min_rtf']


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def param_msgs(**params):
    return [Parameter(k, value=v).to_parameter_msg() for k, v in params.items()]


class Evaluator(Node):
    def __init__(self):
        super().__init__('evaluator', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.scan = None
        self.x = self.y = self.yaw = self.v = 0.0
        self.have_odom = False
        self.wall = None  # (벽면 위 한 점 x, y, 법선 nx, ny), 법선은 로봇에서 벽 쪽
        self.wall_yaw = 0.0
        self.events = []  # (sim time [s], topic, value)
        self.trace = []   # 시도 중 매 순간의 상태. 이상한 결과를 나중에 뜯어보기 위해 남긴다
        self.rtf = None

        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, '/fault_state', lambda m: self.log_event('fault', m.data), 10)
        self.create_subscription(String, '/monitor_state', lambda m: self.log_event('monitor', m.data), 10)
        self.create_subscription(Bool, '/stop_state', lambda m: self.log_event('stop', m.data), 10)
        self.create_subscription(PerformanceMetrics, '/performance_metrics',
                                 lambda m: setattr(self, 'rtf', m.real_time_factor), 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.param_cli = self.create_client(SetParameters, '/fault_injector/set_parameters')
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')

    # ---------- 수신 ----------
    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.v = m.twist.twist.linear.x
        self.have_odom = True

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def log_event(self, topic, value):
        self.events.append((self.now(), topic, value))

    def latest(self, topic):
        for _, tp, v in reversed(self.events):
            if tp == topic:
                return v
        return None

    def first_after(self, t0, topic, pred):
        for t, tp, v in self.events:
            if t >= t0 and tp == topic and pred(v):
                return t
        return None

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def call(self, client, req, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service {client.srv_name} not available')
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    # ---------- 정답 거리 ----------
    def clearance(self):
        """로봇 몸체 앞모서리에서 벽면까지의 실제 거리 [m]."""
        fx, fy, nx, ny = self.wall
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        corners = [(self.x + ROBOT_FRONT_X * c - side * s, self.y + ROBOT_FRONT_X * s + side * c)
                   for side in (ROBOT_HALF_WIDTH, -ROBOT_HALF_WIDTH)]
        return min((fx - px) * nx + (fy - py) * ny for px, py in corners)

    def corridor_clear(self, direction, length, half_width=0.25):
        m = self.scan
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r):
                continue
            a = m.angle_min + i * m.angle_increment - direction
            along, side = r * math.cos(a), r * math.sin(a)
            if 0.0 < along < length and abs(side) < half_width:
                return False
        return True

    # ---------- 조작 ----------
    def publish_cmd(self, v=0.0, w=0.0):
        t = Twist()
        t.linear.x, t.angular.z = v, w
        self.cmd_pub.publish(t)

    def turn_to(self, target, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end:
            err = norm(target - self.yaw)
            if abs(err) < 0.01:
                break
            w = max(-0.5, min(0.5, 1.5 * err))
            self.publish_cmd(0.0, math.copysign(max(abs(w), 0.05), w))
            rclpy.spin_once(self, timeout_sec=0.05)
        self.publish_cmd()
        self.spin_for(0.5)

    def move_to_clearance(self, target, timeout=25.0):
        end = time.time() + timeout
        while time.time() < end:
            err = self.clearance() - target
            if abs(err) < 0.01:
                break
            v = max(-0.15, min(0.15, err))
            self.publish_cmd(math.copysign(max(abs(v), 0.03), v))
            rclpy.spin_once(self, timeout_sec=0.05)
        self.publish_cmd()
        self.spin_for(0.5)

    # ---------- 실험 세팅 ----------
    def setup_wall(self, distance):
        while self.scan is None or not self.have_odom:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.call(self.delete_cli, DeleteEntity.Request(name=WALL_NAME))
        self.spin_for(1.0)
        # 로봇이 지나갈 폭의 통로가 벽 너머까지 빈 방향을 고른다. 옆 장애물에 걸리면 결과가 오염된다
        free = [d for d in range(0, 360, 10) if self.corridor_clear(math.radians(d), distance + 0.3)]
        if not free:
            raise RuntimeError('벽을 세울 빈 통로가 없습니다. empty_world.launch.py에서 실행하세요.')
        best = min(free, key=lambda d: abs(norm(math.radians(d))))  # 지금 방향에서 가장 덜 도는 쪽
        self.turn_to(norm(self.yaw + math.radians(best)))
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        cx, cy = self.x + distance * c, self.y + distance * s
        self.wall = (cx - WALL_THICKNESS / 2 * c, cy - WALL_THICKNESS / 2 * s, c, s)
        self.wall_yaw = self.yaw
        box = f'<box><size>{WALL_THICKNESS} {WALL_WIDTH} 0.4</size></box>'
        sdf = (f"<sdf version='1.6'><model name='{WALL_NAME}'><static>true</static><link name='l'>"
               f"<collision name='c'><geometry>{box}</geometry></collision>"
               f"<visual name='v'><geometry>{box}</geometry><material>"
               f"<ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material></visual>"
               f"</link></model></sdf>")
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = cx, cy, 0.2
        pose.orientation.z, pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        res = self.call(self.spawn_cli, SpawnEntity.Request(name=WALL_NAME, xml=sdf, initial_pose=pose))
        if not res.success:
            raise RuntimeError(f'wall spawn failed: {res.status_message}')
        self.spin_for(1.0)
        self.get_logger().info(f'wall placed, clearance {self.clearance():.2f} m')

    def remove_wall(self):
        self.call(self.delete_cli, DeleteEntity.Request(name=WALL_NAME))

    # ---------- 한 번의 시도 ----------
    def run_trial(self, fault, speed, start, trigger):
        self.call(self.param_cli, SetParameters.Request(parameters=param_msgs(fault='none')))
        self.spin_for(1.5)  # 이전 결함에서 모니터가 정상으로 돌아올 시간
        self.turn_to(self.wall_yaw)
        self.move_to_clearance(start)
        self.turn_to(self.wall_yaw)
        if fault in FAULT_PARAMS:
            self.call(self.param_cli, SetParameters.Request(parameters=param_msgs(**FAULT_PARAMS[fault])))
        self.spin_for(1.0)

        start_clear = self.clearance()
        self.events.clear()
        self.trace = []
        wall0 = time.time()
        inject_clear = inject_time = stop_clear = None
        min_clear = start_clear
        still_since = None
        # 시간 제한은 시뮬레이터 시각으로 잰다. 실제 시각으로 재면 Gazebo가 느려질 때(RTF < 1)
        # 로봇이 아직 달리는 중인데 시도가 끝나서 "멈춤"으로 잘못 기록된다
        sim_deadline = self.now() + start / speed + 8.0
        wall_deadline = time.time() + 120.0  # 시뮬레이터가 아예 멈췄을 때를 위한 안전장치
        timed_out = True

        while time.time() < wall_deadline:
            if self.now() > sim_deadline:
                break
            self.publish_cmd(speed)
            rclpy.spin_once(self, timeout_sec=0.02)
            c = self.clearance()
            min_clear = min(min_clear, c)
            if inject_time is None and c <= trigger:
                inject_clear, inject_time = c, self.now()
                # 응답을 기다리지 않고 보낸다. 실제 적용 시각은 /fault_state로 따로 잰다
                self.param_cli.call_async(SetParameters.Request(parameters=param_msgs(fault=fault)))
            stopping = self.latest('stop') is True
            self.trace.append((round(time.time() - wall0, 3), round(self.now(), 3), round(c, 3), round(self.v, 3),
                               stopping, self.latest('fault'), self.latest('monitor'),
                               round(self.rtf, 2) if self.rtf is not None else ''))
            if inject_time is not None and stopping and stop_clear is None:
                stop_clear = c
            if c < COLLISION_CLEARANCE:
                timed_out = False
                break
            if inject_time is not None and stopping and abs(self.v) < 0.005:
                still_since = still_since or self.now()
                if self.now() - still_since > 1.5:
                    timed_out = False
                    break
            else:
                still_since = None

        self.publish_cmd()
        self.spin_for(0.8)
        min_clear = min(min_clear, self.clearance())

        # 결함 노드가 실제로 결함을 내보내기 시작한 시각을 기준으로 지연을 잰다
        t_fault = t_detect = t_stop = None
        if inject_time is not None:
            t_fault = self.first_after(inject_time, 'fault', lambda v: v == fault) or inject_time
            t_detect = self.first_after(t_fault, 'monitor', lambda v: v != 'OK')
            t_stop = self.first_after(t_fault, 'stop', lambda v: v is True)

        def r3(x):
            return '' if x is None else round(x, 3)

        return {
            'fault': fault, 'speed': speed,
            'start_clearance': r3(start_clear),
            'inject_clearance': r3(inject_clear),
            'detect_latency_s': r3(t_detect - t_fault) if t_detect else '',
            'stop_latency_s': r3(t_stop - t_fault) if t_stop else '',
            'stop_clearance': r3(stop_clear),
            'final_clearance': r3(max(min_clear, 0.0)),
            'outcome': ('collision' if min_clear < COLLISION_CLEARANCE
                        else 'timeout' if timed_out else 'stopped'),
            'min_rtf': min((t[-1] for t in self.trace if t[-1] != ''), default=''),
        }


class LaunchedSafety:
    """safety.launch.py를 띄우고 끄는 도우미."""

    def __init__(self, monitor, log_path):
        self.log = open(log_path, 'w')
        self.proc = subprocess.Popen(
            ['ros2', 'launch', 'lidar_safety', 'safety.launch.py', f'monitor:={str(monitor).lower()}'],
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def stop(self):
        os.killpg(self.proc.pid, signal.SIGINT)
        try:
            self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(self.proc.pid, signal.SIGKILL)
        self.log.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--faults', default='none,delay,zero,freeze,noise')
    ap.add_argument('--speeds', default='0.15,0.22', help='전진 속도 [m/s], burger 최고 0.22')
    ap.add_argument('--repeats', type=int, default=3)
    ap.add_argument('--monitor', default='false,true', help='모니터 끔/켬 조건')
    ap.add_argument('--wall', type=float, default=1.8, help='처음 벽을 세울 거리 [m]')
    ap.add_argument('--start', type=float, default=1.3, help='매 시도 출발 시 벽까지 여유거리 [m]')
    ap.add_argument('--trigger', type=float, default=0.8, help='이 여유거리에 오면 결함 주입 [m]')
    ap.add_argument('--out', default=str(pathlib.Path.home() / 'ros2_ws' / 'eval_results'))
    args = ap.parse_args()

    faults = args.faults.split(',')
    speeds = [float(s) for s in args.speeds.split(',')]
    monitors = [m.strip() == 'true' for m in args.monitor.split(',')]
    out_dir = pathlib.Path(args.out) / datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

    rclpy.init()
    node = Evaluator()
    node.spin_for(2.0)
    if 'fault_injector' in [n for n, _ in node.get_node_names_and_namespaces()]:
        print('safety.launch.py가 이미 떠 있습니다. 끄고 다시 실행하세요 (이 스크립트가 직접 띄웁니다).')
        return

    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / 'trials.csv'
    trace_dir = out_dir / 'traces'
    trace_dir.mkdir()
    total = len(monitors) * len(faults) * len(speeds) * args.repeats
    done = 0
    try:
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            writer.writeheader()
            for monitor in monitors:
                safety = LaunchedSafety(monitor, out_dir / f'nodes_monitor_{"on" if monitor else "off"}.log')
                try:
                    if not node.param_cli.wait_for_service(timeout_sec=20.0):
                        raise RuntimeError('fault_injector did not start')
                    node.spin_for(3.0)
                    if node.wall is None:
                        node.setup_wall(args.wall)
                    for speed in speeds:
                        for fault in faults:
                            for rep in range(1, args.repeats + 1):
                                row = node.run_trial(fault, speed, args.start, args.trigger)
                                row.update(monitor='on' if monitor else 'off', repeat=rep)
                                writer.writerow(row)
                                f.flush()
                                done += 1
                                trace_path = trace_dir / f'{done:03d}_{row["monitor"]}_{fault}_{speed}_{rep}.csv'
                                with open(trace_path, 'w', newline='') as tf:
                                    tw = csv.writer(tf)
                                    tw.writerow(['wall_s', 'sim_s', 'clearance', 'v', 'stop', 'fault', 'monitor', 'rtf'])
                                    tw.writerows(node.trace)
                                print(f'[{done:3d}/{total}] monitor {row["monitor"]:3s} | {fault:6s} | '
                                      f'{speed:.2f} m/s | final {row["final_clearance"]:.2f} m | '
                                      f'{row["outcome"]:9s} | detect {row["detect_latency_s"] or "-"} s | '
                                      f'stop {row["stop_latency_s"] or "-"} s', flush=True)
                finally:
                    node.publish_cmd()
                    safety.stop()
                    node.spin_for(2.0)
    finally:
        if node.wall is not None:
            node.remove_wall()
        node.destroy_node()
        rclpy.try_shutdown()

    report.write_report(csv_path)
    print(f'\n결과: {out_dir}')


if __name__ == '__main__':
    main()
