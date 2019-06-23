# Copyright 2019 Norwegian University of Science and Technology
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
from launch_ros.actions import Node


def generate_launch_description():

    urdf = os.path.join(get_package_share_directory(
        'kuka_kr6_support'), 'urdf', 'kr6r900sixx.urdf')
    assert os.path.exists(urdf)

    rviz_config_dir = os.path.join(get_package_share_directory(
        'kuka_kr6_support'), 'config', 'model.rviz')

    return LaunchDescription([
        Node(package='rviz2',
             node_executable='rviz2',
             node_name='rviz2',
             arguments=['-d', rviz_config_dir],
             output='screen'),

        Node(package='robot_state_publisher',
             node_executable='robot_state_publisher',
             node_name='robot_state_publisher',
             output='screen',
             arguments=[urdf]),

        Node(package='kuka_rsi_simulator',
             node_executable='joint_state_publisher',
             node_name='joint_state_publisher',
             output='screen'),

    ])
