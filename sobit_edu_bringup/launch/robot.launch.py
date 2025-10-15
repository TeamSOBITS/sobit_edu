import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_edu')

    arg_robot_coords_x = DeclareLaunchArgument('robot_coords_x', default_value='0')
    arg_robot_coords_y = DeclareLaunchArgument('robot_coords_y', default_value='0')
    arg_robot_coords_Y = DeclareLaunchArgument('robot_coords_Y', default_value='0')
    
    arg_enable_gz = DeclareLaunchArgument('enable_gz', default_value='True')
    arg_enable_gz_lidar = DeclareLaunchArgument('enable_gz_lidar', default_value='True')
    arg_enable_gz_imu = DeclareLaunchArgument('enable_gz_imu', default_value='True')

    return LaunchDescription([
        arg_robot_name,
        arg_robot_coords_x,
        arg_robot_coords_y,
        arg_robot_coords_Y,
        arg_enable_gz,
        arg_enable_gz_lidar,
        arg_enable_gz_imu,
        OpaqueFunction(function = launch_gz),
    ])


def launch_gz(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    robot_coords_x = LaunchConfiguration('robot_coords_x').perform(context)
    robot_coords_y = LaunchConfiguration('robot_coords_y').perform(context)
    robot_coords_Y = LaunchConfiguration('robot_coords_Y').perform(context)
    enable_gz = LaunchConfiguration('enable_gz').perform(context)
    enable_gz_lidar = LaunchConfiguration('enable_gz_lidar').perform(context)
    enable_gz_imu = LaunchConfiguration('enable_gz_imu').perform(context)
    robot_description = os.path.join(get_package_share_directory(
        'sobit_edu_description'), 
        'robots',
        'sobit_edu.urdf.xacro'
    )
    robot_description_config = xacro.process_file(
        robot_description,
        mappings={
            'enable_gz' : enable_gz,
            'robot_name' : robot_name,
            'enable_gz_lidar' : enable_gz_lidar,
            'enable_gz_imu' : enable_gz_imu,
        })

    if enable_gz == 'False':
        controller_config = os.path.join(
            get_package_share_directory(
                'sobit_edu_control'),
                "config", 
                "controllers.yaml"
        )
        ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_config.toxml()}, controller_config],
            output="screen",
            namespace=robot_name,
        )

    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active',
             '--controller-manager', robot_name+'/controller_manager',
            #  '--use-sim-time',
             'joint_state_broadcaster'
        ],
        output='screen'
    )

    joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active',
             '--controller-manager', robot_name+'/controller_manager',
            #  '--use-sim-time',
             'joint_trajectory_controller'
        ],
        output='screen'
    )

    velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'configured',
             '--controller-manager', robot_name+'/controller_manager',
            #  '--use-sim-time',
             'velocity_controller'
        ],
        output='screen'
    )

    diff_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active',
             '--controller-manager', robot_name+'/controller_manager',
            #  '--use-sim-time',
             'diff_controller'
        ],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        parameters=[
            {"frame_prefix": robot_name + '/'},
            {"robot_description": robot_description_config.toxml()},
            {"use_sim_time": True if enable_gz == 'True' else False},
        ],
        output="screen",
    )

    if enable_gz == 'True':
        gz_spawn_entity_node = Node(
            package='ros_gz_sim',
            executable='create',
            namespace=robot_name,
            arguments=[
                '-topic', '/' + robot_name + '/robot_description',
                '-name', robot_name,
                '-x', robot_coords_x,
                '-y', robot_coords_y,
                '-Y', robot_coords_Y,
            ],
            output='screen',
        )

        gz_bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=robot_name,
            arguments=[
                        "/" + robot_name + "/joint_states" + "@sensor_msgs/msg/JointState" + "[ignition.msgs.Model",
                        # "/model/" + robot_name + "/pose" + "@geometry_msgs/msg/Pose" + "[ignition.msgs.Pose",
                        # "/" + robot_name + "/base_front_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                        # "/" + robot_name + "/base_front_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/base_front_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/base_back_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                        # "/" + robot_name + "/base_back_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/base_back_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/head_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                        # "/" + robot_name + "/head_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/head_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/head_camera/depth/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
                        # "/" + robot_name + "/hand_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                        # "/" + robot_name + "/hand_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/hand_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/hand_camera/depth/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
                        "/" + robot_name + "/lidar/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",

                        "/" + robot_name + "/lidar/scan/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
                        "/" + robot_name + "/imu" + "@sensor_msgs/msg/Imu" + "[ignition.msgs.IMU",
                    ],

            output='screen'
        )
        # gz_tf_head_cam_node = Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['--frame-id', robot_name + '/head_camera_depth_optical_frame',
        #                '--child-frame-id', robot_name + '/head_pitch_link/head_camera_depth',
        #                '--pitch', '-1.57',
        #                '--roll', '1.57'],
        #     output='screen',
        # )

        # gz_tf_hand_cam_node = Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['--frame-id', robot_name + '/hand_camera_depth_optical_frame',
        #                '--child-frame-id', robot_name + '/arm_wrist_roll_link/hand_camera_depth',
        #                '--pitch', '-1.57',
        #                '--roll', '1.57'],
        #     output='screen',
        # )

    action_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sobit_edu_library'),
                'launch',
                'library_server.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name,
            'enable_gz': enable_gz,
        }.items(),
    )

    if enable_gz == 'False':
        return {
            ros2_control_node,
            joint_state_broadcaster,
            velocity_controller,
            joint_trajectory_controller,
            robot_state_publisher_node,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[action_server_launch],
                )
            ),
        }
    
    else:
        return [
            gz_spawn_entity_node,
            gz_bridge_node,
            # gz_tf_head_cam_node,
            # gz_tf_hand_cam_node,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity_node,
                    on_exit=[joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[joint_trajectory_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[velocity_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[diff_controller],
                )
            ),
                    RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=joint_state_broadcaster,
                        on_exit=[action_server_launch],
                    )
                ),
            robot_state_publisher_node,
        ]