cmake_minimum_required(VERSION 3.10.2)
project(sobit_edu_library)

## Compile as C++14, supported in ROS Noetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  geometry_msgs
  nav_msgs
  pybind11_catkin
  sobits_msgs
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  dynamixel_sdk
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  geometry_msgs
  nav_msgs
  pybind11_catkin
  sobits_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} src/sobit_edu_joint_controller.cpp
                            src/sobit_edu_wheel_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS})

pybind_add_module(sobit_edu_module MODULE src/sobit_edu_library.cpp
                                          src/sobit_edu_joint_controller.cpp
                                          src/sobit_edu_wheel_controller.cpp)
add_dependencies(sobit_edu_module ${catkin_EXPORTED_TARGETS})

# add_executable(test_grasp example/test_grasp.cpp
#                         src/sobit_edu_joint_controller.cpp
#                         src/sobit_edu_wheel_controller.cpp)
# target_link_libraries(test_grasp ${catkin_LIBRARIES})
# add_dependencies(test_grasp ${catkin_EXPORTED_TARGETS})
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# install(
#   TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )