"""결함 주입 -> 인지 -> (감지 모니터) -> 정지 판단을 한 번에 띄운다.

  ros2 launch lidar_safety safety.launch.py                    # 모니터 켬, 결함 없음
  ros2 launch lidar_safety safety.launch.py monitor:=false     # 모니터 없이 (비교용)
  ros2 launch lidar_safety safety.launch.py fault:=zero        # 처음부터 결함 주입

teleop은 키 입력 때문에 따로 띄운다.
  ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    monitor = LaunchConfiguration('monitor')
    fault = LaunchConfiguration('fault')
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('monitor', default_value='true'),
        DeclareLaunchArgument('fault', default_value='none'),

        Node(package='lidar_safety', executable='fault_injector', output='screen',
             parameters=[sim_time, {'fault': fault}]),
        Node(package='lidar_safety', executable='obstacle_detector', output='screen',
             parameters=[sim_time, {'scan_topic': '/scan_faulty'}]),
        Node(package='lidar_safety', executable='sensor_monitor', output='screen',
             parameters=[sim_time, {'scan_topic': '/scan_faulty'}],
             condition=IfCondition(monitor)),
        Node(package='lidar_safety', executable='stop_controller', output='screen',
             parameters=[sim_time, {'use_monitor': ParameterValue(monitor, value_type=bool)}]),
    ])
