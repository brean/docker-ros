from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='publishing',
            executable='publishing',
            name='publishing',
            output='screen'
        ),
    ])
