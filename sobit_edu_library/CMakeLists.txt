cmake_minimum_required(VERSION 3.0.2)
project(sobit_edu_library)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  geometry_msgs
  nav_msgs
  pybind11_catkin
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS pybind11_catkin
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/sobit_turtlebot_controller.cpp
                            src/sobit_edu_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

pybind_add_module(sobit_edu_module MODULE src/sobit_turtlebot_controller.cpp
                                                src/sobit_edu_controller.cpp
                                                src/sobit_edu_pybind.cpp)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(test_grasp_edu example/test_grasp.cpp
                        src/sobit_edu_controller.cpp
                        src/sobit_turtlebot_controller.cpp)
target_link_libraries(test_grasp_edu ${catkin_LIBRARIES})