# Copyright (c) 2024 Paolo Forte
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace,Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory

    params_dir = get_package_share_directory('athena_launch')


    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')
    autostart = LaunchConfiguration('autostart')
    use_task_planner = LaunchConfiguration('use_task_planner')
    use_respawn = LaunchConfiguration('use_respawn')
    generate_planning_problem = LaunchConfiguration('generate_planning_problem')

    lifecycle_nodes = ['bt_planner','task_planner_server', 'state_updater_server']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56

    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the athena stack')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the planning stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(params_dir, 'params', 'planning_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')


    declare_use_task_planner_cmd = DeclareLaunchArgument(
        'use_task_planner', 
        default_value= 'True',
        description='Use the task planner if true')
    
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    
    declare_generate_planning_problem_cmd = DeclareLaunchArgument(
        'generate_planning_problem', default_value='True',
        description='Whether to generated the planning problem using VLM.')

    load_nodes = GroupAction(
        actions=[
        
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        
        Node(
                package='athena_bt_planner',
                executable='bt_planner',
                name='bt_planner',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
                
        Node(
                condition=IfCondition(generate_planning_problem),
                package='athena_vlm',
                executable='vlm_api',
                name='vlm_api',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
        
        Node(
                condition=IfCondition(use_task_planner),
                package='athena_planner',
                executable='task_planner_server',
                name='task_planner_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),

        Node(
                condition=IfCondition(use_task_planner),
                package='athena_planner',
                executable='state_updater_server',
                name='state_updater_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),

        Node(
                package='athena_lifecycle_manager',
                executable='task_lifecycle_manager',
                name='lifecycle_manager_task_planner',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes},
                            {'bond_timeout': 25.0}
                            ]),
        ]
    )



    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_task_planner_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_generate_planning_problem_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)


    return ld
