"""Launch the demo P-control driver only (parameters: defaults in driver_node.py).

    ros2 launch thr_driver driver.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='thr_driver', executable='driver', name='driver', output='screen'),
    ])
