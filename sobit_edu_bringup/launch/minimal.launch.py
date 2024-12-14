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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro
import yaml


def generate_launch_description():
    robot_name = "sobit_edu"  ### 各ロボットの名前にする
    bringup_pkg = robot_name + "_bringup"
    description_pkg = robot_name + "_description"
    controller_pkg = robot_name + "_control"
    
    
    ##====================turtle bot2=================================================
    share_dir = get_package_share_directory('kobuki_node')
    # There are two different ways to pass parameters to a non-composed node;
    # either by specifying the path to the file containing the parameters, or by
    # passing a dictionary containing the key -> value pairs of the parameters.
    # When starting a *composed* node on the other hand, only the dictionary
    # style is supported.  To keep the code between the non-composed and
    # composed launch file similar, we use that style here as well.
    params_file = os.path.join(share_dir, "config", "kobuki_node_params.yaml")
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']
    kobuki_ros_node = Node(package='kobuki_node',
                                              executable='kobuki_ros_node',
                                              output='both',
                                              parameters=[params])
    ##==================================================================================

    ##==================urg=============================================================
    # urg_launch_py = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory(bringup_pkg), 'launch'),
    #         'urg.launch.py'])
    #         ) 
    ##==================================================================================

    rviz_config = os.path.join(get_package_share_directory(
        bringup_pkg), "rviz", "real.rviz")
    
    robot_description = os.path.join(get_package_share_directory(
        description_pkg), "robots", robot_name + "_robot.urdf.xacro")
    robot_description_config = \
        xacro.process_file(robot_description, mappings={'enable_gz' : 'False'})

    controller_config = os.path.join(
        get_package_share_directory(
            controller_pkg), "config", "controllers.yaml"
    )


    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_config.toxml()}, controller_config],
        output="screen",
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    velocity_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "-c", "/controller_manager"],
        output="screen",
    )

    joint_trajectory_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description_config.toxml()},
            {"use_sim_time": 'False'},],
        output="screen",
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
    	launch_description,
    	kobuki_ros_node,
        ros2_control_node,
        joint_state_broadcaster_node,
        velocity_controller_node,
        joint_trajectory_controller_node,
        robot_state_publisher_node,
        rviz2_node,
    ])
