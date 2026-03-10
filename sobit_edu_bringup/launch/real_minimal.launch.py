from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = "sobit_edu"
    head_camera_name = "gemini_336" # 'xtion' or 'azure_kinect' or 'gemini_336'

    rviz_config = PathJoinSubstitution([
            FindPackageShare('sobit_edu_bringup'),
            'rviz',
            'real.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("sobit_edu_bringup"),
                    'launch',
                    'robot.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_name'           : robot_name,
                'head_camera_name'     : head_camera_name,
                'enable_gz'            : 'False',
                'enable_mobile_base'   : 'True',
                'enable_head'          : 'True',
                'enable_arm'           : 'True',
                'enable_hand'          : 'True',
            }.items()
        ),
        rviz_node,
    ])
