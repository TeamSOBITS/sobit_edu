import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_move_group_launch
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_edu')
    arg_enable_gz = DeclareLaunchArgument('enable_gz', default_value='True')

    return LaunchDescription([
        arg_robot_name,
        arg_enable_gz,
        OpaqueFunction(function = launch_node),
    ])

def launch_node(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    enable_gz = LaunchConfiguration('enable_gz').perform(context)

    package_name_moveit_config = 'sobit_edu_moveit_config'

    pkg_share_moveit_config = FindPackageShare(package=package_name_moveit_config).find(package_name_moveit_config)

    robot_description = os.path.join(get_package_share_directory('sobit_edu_description'), 'robots','sobit_edu.urdf.xacro')
    srdf_model_path = os.path.join(pkg_share_moveit_config, 'config', 'sobit_edu.srdf')
    moveit_controllers_file_path = os.path.join(pkg_share_moveit_config, 'config', 'moveit_controllers.yaml')
    joint_limits_file_path = os.path.join(pkg_share_moveit_config, 'config', 'joint_limits.yaml')
    kinematics_file_path = os.path.join(pkg_share_moveit_config, 'config', 'kinematics.yaml')
    pilz_cartesian_limits_file_path = os.path.join(pkg_share_moveit_config, 'config', 'pilz_cartesian_limits.yaml')
    rviz_config_file = os.path.join(pkg_share_moveit_config, 'rviz', 'moveit.rviz')

    moveit_config = (MoveItConfigsBuilder("sobit_edu", package_name=package_name_moveit_config)
                    .robot_description(file_path="config/sobit_edu.urdf.xacro")
                    .robot_description_semantic(file_path=srdf_model_path)
                    .robot_description_kinematics(file_path=kinematics_file_path)
                    .joint_limits(file_path=joint_limits_file_path)
                    .trajectory_execution(file_path=moveit_controllers_file_path)
                    .planning_scene_monitor(
                        publish_planning_scene=True,
                        publish_geometry_updates=True,
                        publish_state_updates=True,
                        publish_transforms_updates=True,
                        publish_robot_description=False,
                        publish_robot_description_semantic=True,
                    )
                    .planning_pipelines(
                        default_planning_pipeline="ompl",
                        pipelines=["ompl", "pilz_industrial_motion_planner", "chomp", "stomp"],
                        # load_all=True
                    )
                    .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
                    .to_moveit_configs())


    # Add frame_prefix so MoveIt maps URDF frames to TF frames
    config_dict = moveit_config.to_dict()

    octomap_config = {
        'octomap_frame': 'sobit_edu/base_footprint',  # if mobile robot, should be a fixed frame in the world
        'octomap_resolution': 0.05,
        'max_range': 5.0,
    }

    # move_group node
    start_move_group_node_cmd = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=robot_name,
        output="screen",
        parameters=[
            config_dict,
            octomap_config,
            {'use_sim_time': True if enable_gz == 'True' else False},
            {'trajectory_execution.control_multi_dof_joint_variables': True},
            {'robot_description_planning.frame_prefix': robot_name + '/'},
            # 🌟 OMPLにマルチ自由度関節を強制的にサポートさせる設定を追加
            {'robot_description_planning.default_velocity_scaling_factor': 0.1},
            {'robot_description_planning.default_acceleration_scaling_factor': 0.1},
        ],
        remappings=[
            ('/attached_collision_object', 'attached_collision_object'),
            ('/trajectory_execution_event', 'trajectory_execution_event'),
            ('/transform_listener', 'transform_listener'),
            ('/planning_scene', 'planning_scene'),
            ('/planning_scene_world', 'planning_scene_world'),
            ('/recognize_objects', 'recognize_objects'),
            ('/recognized_object_array', 'recognized_object_array'),
            ('/collision_object', 'collision_object'),
            ('/joint_states', 'joint_states'),
            ('tf', '/tf'),
            ('tf_static', '/tf_static')
        ]
    )


    # RViz
    start_rviz_node_cmd = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            # moveit_config.sensors_3d,
            {
                'use_sim_time': True if enable_gz == 'True' else False,
                'robot_description_planning.frame_prefix': robot_name + '/',
            },

        ],
        remappings=[
            ('/attached_collision_object', '/sobit_edu/attached_collision_object'),
            ('/trajectory_execution_event', '/sobit_edu/trajectory_execution_event'),
            ('/transform_listener', '/sobit_edu/transform_listener'),
            ('/planning_scene', '/sobit_edu/planning_scene'),
            ('/planning_scene_world', '/sobit_edu/planning_scene_world'),
            ('/recognize_objects', '/sobit_edu/recognize_objects'),
            ('/recognized_object_array', '/sobit_edu/recognized_object_array'),
            ('/collision_object', '/sobit_edu/collision_object'),
            ('/joint_states', '/sobit_edu/joint_states'),
            ('tf', '/tf'),
            ('tf_static', '/tf_static')
        ]
    )


    launch_nodes = [
        start_move_group_node_cmd,
        start_rviz_node_cmd,
    ]


    return launch_nodes
                    
