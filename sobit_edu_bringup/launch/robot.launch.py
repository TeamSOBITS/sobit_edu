import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import yaml
import xacro

def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_edu')
    arg_head_camera = DeclareLaunchArgument('head_camera_name', default_value='gemini_336')

    arg_robot_coords_x = DeclareLaunchArgument('robot_coords_x', default_value='0')
    arg_robot_coords_y = DeclareLaunchArgument('robot_coords_y', default_value='0')
    arg_robot_coords_Y = DeclareLaunchArgument('robot_coords_Y', default_value='0')

    arg_enable_mobile_base = DeclareLaunchArgument('enable_mobile_base', default_value='True')
    arg_enable_head        = DeclareLaunchArgument('enable_head', default_value='True')
    arg_enable_arm         = DeclareLaunchArgument('enable_arm', default_value='True')
    arg_enable_hand        = DeclareLaunchArgument('enable_hand', default_value='True')

    arg_enable_gz                = DeclareLaunchArgument('enable_gz', default_value='True')
    arg_enable_gz_lidar          = DeclareLaunchArgument('enable_gz_lidar', default_value='True')
    arg_enable_gz_imu            = DeclareLaunchArgument('enable_gz_imu', default_value='True')
    arg_enable_gz_head_cam_color = DeclareLaunchArgument('enable_gz_head_cam_color', default_value='True')
    arg_enable_gz_head_cam_depth = DeclareLaunchArgument('enable_gz_head_cam_depth', default_value='True')

    return LaunchDescription([
        arg_robot_name,
        arg_head_camera,
        arg_robot_coords_x,
        arg_robot_coords_y,
        arg_robot_coords_Y,
        arg_enable_mobile_base,
        arg_enable_head,
        arg_enable_arm,
        arg_enable_hand,
        arg_enable_gz,
        arg_enable_gz_lidar,
        arg_enable_gz_imu,
        arg_enable_gz_head_cam_color,
        arg_enable_gz_head_cam_depth,
        OpaqueFunction(function = launch_gz),
    ])


def launch_gz(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    head_camera_name = LaunchConfiguration('head_camera_name').perform(context)

    robot_coords_x = LaunchConfiguration('robot_coords_x').perform(context)
    robot_coords_y = LaunchConfiguration('robot_coords_y').perform(context)
    robot_coords_Y = LaunchConfiguration('robot_coords_Y').perform(context)

    enable_mobile_base = LaunchConfiguration('enable_mobile_base').perform(context)
    enable_head        = LaunchConfiguration('enable_head').perform(context)
    enable_arm         = LaunchConfiguration('enable_arm').perform(context)
    enable_hand        = LaunchConfiguration('enable_hand').perform(context)

    enable_gz                = LaunchConfiguration('enable_gz').perform(context)
    enable_gz_lidar          = LaunchConfiguration('enable_gz_lidar').perform(context)
    enable_gz_imu            = LaunchConfiguration('enable_gz_imu').perform(context)
    enable_gz_head_cam_color = LaunchConfiguration('enable_gz_head_cam_color').perform(context)
    enable_gz_head_cam_depth = LaunchConfiguration('enable_gz_head_cam_depth').perform(context)

    # Find Dynamixel Port name and Kobuki Port name from DXL_SE_PORT/KOBUKI_SE_PORT environment variable
    dxl_se_port = ''
    kobuki_se_port = ''
    if enable_gz == 'False':
        dxl_se_port = str(os.environ.get('DXL_SE_PORT'))
        print('Dynamixel SOBIT EDU Port : ' + dxl_se_port)
        kobuki_se_port = str(os.environ.get('KOBUKI_SE_PORT'))
        print('Kobuki SOBIT EDU Port : ' + kobuki_se_port)

    robot_description = os.path.join(get_package_share_directory(
        'sobit_edu_description'), 
        'robots',
        'sobit_edu.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        robot_description,
        mappings={
            'enable_mobile_base'       : enable_mobile_base,
            'enable_head'              : enable_head,
            'enable_arm'               : enable_arm,
            'enable_hand'              : enable_hand,
            'enable_gz'                : enable_gz,
            'robot_name'               : robot_name,
            'enable_gz_lidar'          : enable_gz_lidar,
            'enable_gz_imu'            : enable_gz_imu,
            'head_camera_name'         : head_camera_name,
            'enable_gz_head_cam_color' : enable_gz_head_cam_color,
            'enable_gz_head_cam_depth' : enable_gz_head_cam_depth,
            'dxl_se_port' : dxl_se_port,
        })


    urg_config = os.path.join(get_package_share_directory("sobit_edu_bringup"), "config", "urg_node_params.yaml")

    kobuki_param_file = os.path.join(get_package_share_directory("sobit_edu_bringup"), "config", "kobuki_node_params.yaml")
    with open(kobuki_param_file, "r") as f:
        kobuki_params = yaml.safe_load(f)["kobuki_ros_node"]["ros__parameters"]
    kobuki_params["device_port"] = kobuki_se_port


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
            namespace=robot_name,
            parameters=[controller_config],
            remappings=[
                ("controller_manager/robot_description", "robot_description"),
            ],
            output="both",
        )
        
        kobuki_node = Node(
            package="kobuki_node",
            executable="kobuki_ros_node",
            namespace=robot_name,
            output="both",
            parameters=[kobuki_params]
        )
        
        urg_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('urg_node'),
                    'launch',
                    'urg.launch.py'
                ])
            ]),
            launch_arguments={
                "config_file" : urg_config,
                "use_namespace" : "true",
                "namespace" : robot_name,
            }.items()
        )

        if (head_camera_name == "xtion"):
            camera_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([os.path.join(
                        get_package_share_directory('sobit_edu_bringup'),
                        'launch',
                        'xtion.launch.py')
                    ])
                ]),
                launch_arguments={
                    'tf_prefix': robot_name,
                    'namespace': 'head_camera',
                }.items()
            )
        elif (head_camera_name == "azure_kinect"):
            camera_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([os.path.join(
                        get_package_share_directory('sobit_edu_bringup'),
                        'launch',
                        'azure_kinect.launch.py')
                    ])
                ]),
                launch_arguments={
                    'namespace': robot_name,
                }.items()
            )
        elif (head_camera_name == "gemini_336"):
            camera_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([os.path.join(
                        get_package_share_directory('sobit_edu_bringup'),
                        'launch',
                        'gemini_bringup.launch.py')
                    ])
                ]),
                launch_arguments={
                    'namespace': robot_name,
                }.items()
            )
        else:
            camera_node = None

    controllers = []
    nodes = []

    if enable_head == 'True':
        head_position_controller = Node(
            package='controller_manager',
            executable='spawner',
            # name='head_position_controller',
            namespace=robot_name,
            arguments=[
                'head_position_controller',
                '-c', 'controller_manager', '--activate'
                ],
        )
        controllers.append(head_position_controller)

    if enable_arm == 'True':
        arm_position_controller = Node(
            package='controller_manager',
            executable='spawner',
            # name='arm_position_controller',
            namespace=robot_name,
            arguments=[
                'arm_position_controller',
                '-c', 'controller_manager', '--activate'
                ],
        )
        controllers.append(arm_position_controller)

    if enable_hand == 'True':
        hand_position_controller = Node(
            package='controller_manager',
            executable='spawner',
            # name='hand_position_controller',
            namespace=robot_name,
            arguments=[
                'hand_position_controller',
                '-c', 'controller_manager', '--activate'
                ],
        )
        controllers.append(hand_position_controller)


    if enable_mobile_base == 'True' and enable_gz == 'True':
        wheel_controller = Node(
            package='controller_manager',
            executable='spawner',
            # name='wheel_controller',
            namespace=robot_name,
            arguments=[
                'wheel_controller',
                '-c', 'controller_manager', '--activate'
                ],
        )
        controllers.append(wheel_controller)

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        # name='joint_state_broadcaster',
        namespace=robot_name,
        arguments=[
            'joint_state_broadcaster',
            '-c', 'controller_manager',
            ],
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
                        "/" + robot_name + "/joint_states" + "@sensor_msgs/msg/JointState" + "[gz.msgs.Model",
                        # "/model/" + robot_name + "/pose" + "@geometry_msgs/msg/Pose" + "[gz.msgs.Pose",
                        # "/" + robot_name + "/base_front_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[gz.msgs.CameraInfo",
                        # "/" + robot_name + "/base_front_camera/color" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                        # "/" + robot_name + "/base_front_camera/depth" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                        # "/" + robot_name + "/base_back_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[gz.msgs.CameraInfo",
                        # "/" + robot_name + "/base_back_camera/color" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                        # "/" + robot_name + "/base_back_camera/depth" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                        "/" + robot_name + "/head_camera_base/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[gz.msgs.CameraInfo",
                        "/" + robot_name + "/head_camera_base/color" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                        "/" + robot_name + "/head_camera_base/depth" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                        "/" + robot_name + "/head_camera_base/depth/points" + "@sensor_msgs/msg/PointCloud2" + "[gz.msgs.PointCloudPacked",
                        "/" + robot_name + "/scan" + "@sensor_msgs/msg/LaserScan" + "[gz.msgs.LaserScan",

                        # "/" + robot_name + "/scan/points" + "@sensor_msgs/msg/PointCloud2" + "[gz.msgs.PointCloudPacked",
                        "/" + robot_name + "/imu" + "@sensor_msgs/msg/Imu" + "[gz.msgs.IMU",
                    ],
            output='screen'
        )
        
        if (head_camera_name == "xtion"):
            gz_tf_head_cam_node = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['--frame-id', robot_name + '/head_camera_depth_optical_frame',
                        '--child-frame-id', robot_name + '/head_camera_tilt_link/head_camera_depth',
                        '--pitch', '-1.57',
                        '--roll', '1.57'],
                output='screen',
            )
        elif (head_camera_name == "azure_kinect"):
            gz_tf_head_cam_node = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['--frame-id', robot_name + '/head_camera_depth_optical_link',
                        '--child-frame-id', robot_name + '/head_camera_tilt_link/head_camera_depth',
                        '--pitch', '-1.57',
                        '--roll', '1.57'],
                output='screen',
            )
        elif (head_camera_name == "gemini_336"):
            gz_tf_head_cam_node = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['--frame-id', robot_name + '/head_camera_camera_depth_optical_frame',
                        '--child-frame-id', robot_name + '/head_camera_tilt_link/head_camera_depth',
                        '--pitch', '-1.57',
                        '--roll', '1.57'],
                output='screen',
            )
        else:
            gz_tf_head_cam_node = None

        delayed_joint_state_broadcaster = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity_node,
                on_exit=joint_state_broadcaster,
            )
        )

        delayed_controllers = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=controllers,
            )
        )

        vel_remap_node = Node(
            package="twist_stamper",
            executable="twist_stamper",
            namespace=robot_name,
            name="vel_remap",
            arguments=["-r", f"cmd_vel_in:=/{robot_name}/commands/velocity", "-r", f"cmd_vel_out:=/{robot_name}/wheel_controller/cmd_vel", "-p", f"frame_id:={robot_name}/base_footprint"]
        )

        delayed_vel_remap_node = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[vel_remap_node],
            )
        )

        odom_remap_node = Node(
            package="topic_tools",
            executable="relay",
            name="odom_remap",
            arguments=[f"/{robot_name}/wheel_controller/odom", f"/{robot_name}/odom"]
        )

        delayed_odom_remap_node = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[odom_remap_node],
            )
        )

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

    delayed_action_server_launch = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[action_server_launch],
        )
    )

    if enable_gz == 'True':
        nodes.append(gz_bridge_node)
        nodes.append(gz_spawn_entity_node)
        nodes.append(delayed_joint_state_broadcaster)
        nodes.append(delayed_vel_remap_node)
        nodes.append(delayed_odom_remap_node)
        nodes.append(delayed_controllers)
        nodes.append(gz_tf_head_cam_node)
    else:
        nodes.append(ros2_control_node)
        nodes.append(joint_state_broadcaster)
        nodes.extend(controllers)
        nodes.append(kobuki_node)
        nodes.append(urg_node)
        nodes.append(camera_node)

    nodes.append(robot_state_publisher_node)
    nodes.append(delayed_action_server_launch)

    return nodes
