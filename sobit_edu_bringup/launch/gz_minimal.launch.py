import os
import sys
import tempfile
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# sobits_gazebo_worlds/scripts is installed under its own share dir (see
# that package's CMakeLists.txt), so it can be imported cross-package the
# same way sobits_gazebo_worlds/launch/world.launch.py imports it locally.
_SOBITS_WORLDS_SCRIPTS_DIR = os.path.join(
    get_package_share_directory('sobits_gazebo_worlds'), 'scripts')
if _SOBITS_WORLDS_SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SOBITS_WORLDS_SCRIPTS_DIR)

from launch_utils import build_gz_resource_path  # noqa: E402


def generate_launch_description():
    world_model = 'rcjo2025' # empty, wrs, small_house, rcjo2025

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                    "/clock" + "@rosgraph_msgs/msg/Clock" + "[gz.msgs.Clock",
                    "/tf" + "@tf2_msgs/msg/TFMessage" + "[gz.msgs.Pose_V",
                   ],
        output='screen'
    )

    world_file = ''
    if world_model == 'empty':
        world_file = os.path.join(get_package_share_directory(
            'sobit_edu_description'),
            'worlds',
            'empty_w_physics.sdf'
        )
    elif world_model == 'wrs':
        world_file = os.path.join(get_package_share_directory(
            'tmc_wrs_gz_worlds'),
            'worlds',
            'wrs2020.world.xacro'
        )
    elif world_model == 'small_house':
        world_file = os.path.join(get_package_share_directory(
            'aws_small_house_world'),
            'worlds',
            'small_house.world'
        )
    elif world_model == 'rcjo2025':
        rcjo2025_xacro = os.path.join(get_package_share_directory(
            'sobits_gazebo_worlds'),
            'worlds',
            'rcjo2025_arena.world.xacro'
        )
        # EduRobotManager's 表示/非表示 (setRobotVisible()) needs
        # edu_robot_freeze_system attached to the world -- see
        # EduRobotFreezeSystem.cc. rcjo2025_arena.world.xacro is shared
        # with sobit_mini/sobit_light/sobit_pro's own gz_minimal.launch.py
        # and none of them ship that plugin, so it must not be added to
        # that file itself (or their gz sim would fail to find it at
        # startup) -- everything sobit_edu-specific stays in this
        # package. Expand the xacro exactly as sobits_gazebo_worlds wrote
        # it (no args overridden), then splice the plugin into the
        # resulting <world> element here instead.
        expanded = xacro.process_file(rcjo2025_xacro).toxml()
        world_root = ET.fromstring(expanded)
        ET.SubElement(
            world_root.find('world'), 'plugin',
            filename='edu_robot_freeze_system',
            name='edu_gz_gui::EduRobotFreezeSystem')
        world_file = os.path.join(
            tempfile.gettempdir(), 'sobit_edu_rcjo2025_arena.sdf')
        with open(world_file, 'w') as sdf_file:
            sdf_file.write(ET.tostring(world_root, encoding='unicode'))

    # gui.config wires up two, separately-docked panel groups. Neither one
    # pre-spawns anything -- EDU only appears once "Spawn" is pressed in
    # EduRobotManager, same as a human only appears once spawned from
    # HumanControlPanel.
    #  - EduRobotManager + EduOperationLog (sobit_edu_gz_gui, this robot's
    #    own package -- no dependency on guider_multifloor_builder):
    #    spawn/show-hide/rviz2/delete, camera follow, and slider+button
    #    teleop for SOBIT EDU. setRobotVisible(false) actually stops the
    #    ros2 launch process group (nodes/topics really go away), not just
    #    a cosmetic hide -- see EduRobotManager.cc.
    #  - gz_human_sim's HumanControlPanel: human spawn/teleop, same plugin
    #    sobits_gazebo_worlds/config/gui.config uses.
    # Both plugin dirs sit outside the default gz-gui plugin search path,
    # so GZ_GUI_PLUGIN_PATH has to point at them explicitly -- same fix as
    # sobits_gazebo_worlds/launch/world.launch.py, needed here again
    # because that fix is local to that launch file's own process tree.
    # EduRobotManager's hide/show also needs its companion gz-sim System
    # plugin (edu_robot_freeze_system, spliced into the rcjo2025 world
    # above) to be found via GZ_SIM_SYSTEM_PLUGIN_PATH.
    gui_config_path = os.path.join(
        get_package_share_directory('sobit_edu_bringup'), 'config', 'gui.config')
    human_gui_plugin_dir = os.path.join(
        get_package_prefix('gz_human_sim'), 'lib', 'gz_human_sim', 'gz-gui')
    edu_gz_gui_prefix = get_package_prefix('sobit_edu_gz_gui')
    edu_gui_plugin_dir = os.path.join(
        edu_gz_gui_prefix, 'lib', 'sobit_edu_gz_gui', 'gz-gui')
    edu_system_plugin_dir = os.path.join(
        edu_gz_gui_prefix, 'lib', 'sobit_edu_gz_gui', 'gz-sim')
    gui_plugin_path = human_gui_plugin_dir + os.pathsep + edu_gui_plugin_dir

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=build_gz_resource_path(
                get_package_share_directory('sobits_gazebo_worlds')
            ),
        ),
        SetEnvironmentVariable(
            name='GZ_GUI_PLUGIN_PATH',
            value=gui_plugin_path + os.pathsep + os.environ.get('GZ_GUI_PLUGIN_PATH', ''),
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=edu_system_plugin_dir + os.pathsep + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
        ),
        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=gui_plugin_path + os.pathsep + edu_system_plugin_dir + os.pathsep
                + os.environ.get('LD_LIBRARY_PATH', ''),
        ),
        # Hybrid-graphics workaround, same as world.launch.py: without this,
        # gz-sim's ogre2 render pass can silently fall back to the display's
        # default GLX vendor (llvmpipe software rendering on some hybrid
        # Intel/NVIDIA laptops) instead of the discrete GPU.
        SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia'),
        SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1'),
        # Launch gazebo environment. No robot is included here -- spawn it
        # from the EduRobotManager panel's "Spawn" button once the GUI is up.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={
                'gz_args' : ' -r -v 4 ' + world_file + ' --gui-config ' + gui_config_path,
            }.items()
        ),
        gz_bridge_node,
    ])
