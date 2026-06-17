
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='athena_lm',
            executable='problem_file_generator',
            name='problem_file_generator_node',
            output='screen',
            parameters=[],
        ),
    ])
