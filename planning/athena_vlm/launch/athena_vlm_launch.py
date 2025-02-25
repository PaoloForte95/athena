
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='athena_vlm',
            executable='vlm_api',
            name='vlm_api',
            output='screen',
            parameters=[],
        ),
    ])
