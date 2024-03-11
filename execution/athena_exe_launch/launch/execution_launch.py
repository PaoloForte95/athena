# Copyright (c) 2023 Paolo Forte
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the launch directory

    exe_launch_dir = get_package_share_directory('athena_exe_launch')


    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')

    
    robots = [{'name': 'robot1'} ]


    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='True',
        description='Whether to apply a namespace to the planning stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(exe_launch_dir, 'params', 'executor_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    execution_launch_cmds = []
    for robot in robots:
        execution_launch_cmds.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(exe_launch_dir, 'launch', 'execution.py')),
        launch_arguments={'params_file': params_file,
                          'log_level': log_level,
                          'use_sim_time': use_sim_time,
                          'use_namespace': use_namespace,
                          'namespace': robot['name'],
                          'autostart': autostart,
                          'use_respawn': use_respawn
                          }.items()))


    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_respawn_cmd)
    # Add the actions to launch all of the navigation nodes
    for execution_launch_cmd in execution_launch_cmds:
        ld.add_action(execution_launch_cmd)


    return ld
