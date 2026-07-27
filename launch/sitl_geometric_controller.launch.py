#!/usr/bin/env python3
#
# Copyright 2026 Chaoheng Meng
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
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_bool(value):
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def expand_user_path(path):
    return os.path.abspath(os.path.expandvars(os.path.expanduser(path)))


def launch_setup(context):
    px4_dir = expand_user_path(LaunchConfiguration('px4_dir').perform(context))
    px4_model = LaunchConfiguration('px4_model').perform(context)
    agent_port = LaunchConfiguration('agent_port').perform(context)
    headless = launch_bool(LaunchConfiguration('headless').perform(context))
    start_agent = launch_bool(LaunchConfiguration('start_agent').perform(context))
    px4_env = {'HEADLESS': '1'} if headless else None

    package_share = get_package_share_directory('geometric_controller')
    controller_launch = os.path.join(package_share, 'launch', 'geometric_controller.launch.py')

    actions = [
        ExecuteProcess(
            cmd=['make', 'px4_sitl', px4_model],
            cwd=px4_dir,
            output='screen',
            additional_env=px4_env,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller_launch),
            launch_arguments={
                'param_file': LaunchConfiguration('param_file'),
                'launch_rviz': LaunchConfiguration('launch_rviz'),
                'launch_tuning_panel': LaunchConfiguration('launch_tuning_panel'),
                'rviz_config': LaunchConfiguration('rviz_config'),
            }.items(),
        ),
    ]

    if start_agent:
        actions.insert(
            1,
            ExecuteProcess(
                cmd=['MicroXRCEAgent', 'udp4', '-p', agent_port],
                output='screen',
            ),
        )

    return actions


def generate_launch_description():
    package_share = get_package_share_directory('geometric_controller')
    default_params = os.path.join(package_share, 'config', 'controller.yaml')
    default_rviz = os.path.join(package_share, 'rviz', 'geometric_controller.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_dir',
            default_value='~/PX4-Autopilot',
            description='PX4-Autopilot checkout path. Override with px4_dir:=/path/to/PX4-Autopilot.',
        ),
        DeclareLaunchArgument(
            'px4_model',
            default_value='gz_iris',
            description='PX4 SITL make target model.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Set HEADLESS=1 for PX4 Gazebo SITL.',
        ),
        DeclareLaunchArgument(
            'agent_port',
            default_value='8888',
            description='Micro XRCE-DDS Agent UDP port.',
        ),
        DeclareLaunchArgument(
            'start_agent',
            default_value='true',
            description='Start Micro XRCE-DDS Agent from this launch file.',
        ),
        DeclareLaunchArgument(
            'param_file',
            default_value=default_params,
            description='Main YAML parameter file for trajectory_offboard_node.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Start RViz2 with the package visualization config.',
        ),
        DeclareLaunchArgument(
            'launch_tuning_panel',
            default_value='true',
            description='Start the C++/Qt trajectory parameter panel.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='RViz config file.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
