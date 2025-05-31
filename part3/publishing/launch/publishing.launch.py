from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_watch_path = DeclareLaunchArgument(
        'code_watch_directory',
        default_value='/ws/publishing/',
        description='Directory to watch for code changes.'
    )

    return LaunchDescription([
        declared_watch_path,
        Node(
            package='publishing',
            executable='publishing',
            name='publishing',
            output='screen',
            respawn=True,
            parameters=[
                {'watch_path': LaunchConfiguration('code_watch_directory')}
            ]
        ),
    ])
