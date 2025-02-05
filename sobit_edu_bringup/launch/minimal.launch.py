# Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
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

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


from ament_index_python.packages import get_package_share_directory

import yaml 
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.actions import Node
# import xacro
# import yaml
# import launch_ros
# from launch.actions import IncludeLaunchDescription,SetLaunchConfiguration,DeclareLaunchArgument,LogInfo




def generate_launch_description():
    robot_name = "sobit_edu"
    bringup_pkg = robot_name + "_bringup"
    rviz_config = os.path.join(get_package_share_directory(
        bringup_pkg), "rviz", "real.rviz")
    
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        # robot_state_publisher_node,
        rviz2_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sobit_edu_bringup'),
                    'launch',
                    'robot.launch.py'
                ])

            ]),
            launch_arguments={
                'robot_name': 'sobit_edu',
                'robot_coords_x': '0', # x 
                'robot_coords_y': '0', # y
                'robot_coords_Y': '0', # yaw
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('kobuki_node'),
                    'launch',
                    'kobuki_node-launch.py'
                ])

            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('urg_node'),
                    'launch',
                    'urg_node_launch.py'
                ])

            ]),
        )
    ])
