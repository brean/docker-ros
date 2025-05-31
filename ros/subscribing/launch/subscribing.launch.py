from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='subscribing',
            executable='subscribing',
            name='subscribing',
            output='screen'
        ),
    ])
