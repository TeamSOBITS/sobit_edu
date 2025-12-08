from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    use_gui = LaunchConfiguration('use_gui', default='True')

    robot_name = "sobit_edu"
    # head_camera_name = "xtion" ## "xtion" or "gemini_336" # TODO

    rviz_config = PathJoinSubstitution([FindPackageShare('sobit_edu_description'), 'rviz', 'display.rviz'])

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(name='jsp_gui', default_value='true', choices=['true', 'false'],
                                        description='Flag to enable joint_state_publisher_gui'))

    # need to manually pass configuration in because of https://github.com/ros2/launch/issues/313
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': "sobit_edu_description",
            'urdf_package_path': PathJoinSubstitution(['robots', robot_name+'.urdf.xacro'])}.items()
    ))

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(use_gui)
    ))

    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui)
    ))

    # Visualize robot in rviz    
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    ))
    return ld
