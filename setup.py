from setuptools import find_packages, setup

package_name = 'lidar_safety'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/safety.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soomin',
    maintainer_email='minsoominsoo@inha.edu',
    description='TurtleBot3 lidar fault injection, obstacle stop, and sensor fault monitor',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'obstacle_detector = lidar_safety.obstacle_detector:main',
            'stop_controller = lidar_safety.stop_controller:main',
            'fault_injector = lidar_safety.fault_injector:main',
            'sensor_monitor = lidar_safety.sensor_monitor:main',
            'evaluate = lidar_safety.evaluate:main',
            'eval_report = lidar_safety.report:main',
        ],
    },
)
