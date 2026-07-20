cmake_minimum_required(VERSION 3.16)
project(sobit_edu_gz_gui)

# Let AUTOMOC process .hh headers (matches guider_multifloor_builder's
# convention, which this package's GUI plugins were ported from).
if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core)
find_package(Threads REQUIRED)

# World-level system: while EduRobotManager has a robot 非表示/削除
# ("hidden"), its gz entity is kept alive rather than actually removed
# (see EduRobotFreezeSystem.cc for why) -- this system parks it far below
# the world and holds that pose every physics step.
add_library(edu_robot_freeze_system SHARED
  src/EduRobotFreezeSystem.cc
)
target_compile_features(edu_robot_freeze_system PUBLIC cxx_std_17)
set_target_properties(edu_robot_freeze_system PROPERTIES
  OUTPUT_NAME edu_robot_freeze_system
  INSTALL_RPATH_USE_LINK_PATH TRUE
)
target_link_libraries(edu_robot_freeze_system
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::core
  gz-msgs10::core
)

# GUI plugin: spawns/removes/hides/shows SOBIT EDU (runs
# sobit_edu_bringup/robot.launch.py as a child process), toggles its
# sensor bridges, drives the GUI camera's follow/track targets, and
# teleoperates it via gz.msgs.Twist on /<name>/cmd_vel (bridged to ROS by
# robot.launch.py itself).
add_library(EduRobotManager SHARED
  src/EduRobotManager.cc
  src/EduRobotManager.hh
  gui/EduRobotManager.qrc
)
target_compile_features(EduRobotManager PUBLIC cxx_std_17)
set_target_properties(EduRobotManager PROPERTIES
  OUTPUT_NAME EduRobotManager
  INSTALL_RPATH_USE_LINK_PATH TRUE
  AUTOMOC ON
  AUTORCC ON
)
target_link_libraries(EduRobotManager
  gz-gui8::gz-gui8
  gz-rendering8::gz-rendering8
  gz-transport13::core
  gz-msgs10::core
  Qt5::Core
  Threads::Threads
)

# Sidebar panel showing EduRobotManager's operation log.
add_library(EduOperationLog SHARED
  src/EduOperationLog.cc
  src/EduOperationLog.hh
  gui/EduOperationLog.qrc
)
target_compile_features(EduOperationLog PUBLIC cxx_std_17)
set_target_properties(EduOperationLog PROPERTIES
  OUTPUT_NAME EduOperationLog
  INSTALL_RPATH_USE_LINK_PATH TRUE
  AUTOMOC ON
  AUTORCC ON
)
target_link_libraries(EduOperationLog
  gz-gui8::gz-gui8
  gz-transport13::core
  gz-msgs10::core
  Qt5::Core
)

# colcon --symlink-install points the installed program back to this
# source file. Guarantee executable permission before creating that
# symlink.
file(CHMOD
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/edu_teleop_keyboard.py
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

set(EDU_SYSTEM_PLUGIN_DIR lib/${PROJECT_NAME}/gz-sim)
set(EDU_GUI_PLUGIN_DIR lib/${PROJECT_NAME}/gz-gui)

# Keep the gz-sim System plugin and gz-gui plugins in separate search
# paths (GZ_SIM_SYSTEM_PLUGIN_PATH vs GZ_GUI_PLUGIN_PATH), same layout
# guider_multifloor_builder used.
install(TARGETS edu_robot_freeze_system
  ARCHIVE DESTINATION ${EDU_SYSTEM_PLUGIN_DIR}
  LIBRARY DESTINATION ${EDU_SYSTEM_PLUGIN_DIR}
)

install(TARGETS EduRobotManager EduOperationLog
  ARCHIVE DESTINATION ${EDU_GUI_PLUGIN_DIR}
  LIBRARY DESTINATION ${EDU_GUI_PLUGIN_DIR}
)

install(PROGRAMS
  scripts/edu_teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks(env-hooks/sobit_edu_gz_gui.sh.in)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
