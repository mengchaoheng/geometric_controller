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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('geometric_controller')
    default_params = os.path.join(package_share, 'config', 'controller.yaml')
    default_rviz = os.path.join(package_share, 'rviz', 'geometric_controller.rviz')

    return LaunchDescription([
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
        Node(
            package='geometric_controller',
            executable='trajectory_offboard_node',
            name='trajectory_offboard_node',
            output='screen',
            parameters=[LaunchConfiguration('param_file')],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('launch_tuning_panel')),
            package='geometric_controller',
            executable='trajectory_control_panel',
            name='trajectory_control_panel',
            output='screen',
            parameters=[{'target_node': '/trajectory_offboard_node'}],
        ),
    ])
