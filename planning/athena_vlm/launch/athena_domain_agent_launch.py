from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():


    pkg_share = get_package_share_directory("athena_vlm")

    param_file = os.path.join(
        pkg_share,
        "params",
        "vlm_params.yaml"
    )

    return LaunchDescription([
        Node(
            package="athena_vlm",
            executable="capability_extractor_agent",
            name="capability_extractor_agent_node",
            parameters=[param_file],
            output="screen"
        ),
        Node(
            package="athena_vlm",
            executable="pddl_domain_agent",
            name="pddl_domain_agent_node",
            parameters=[param_file],
            output="screen",
        )
    ])