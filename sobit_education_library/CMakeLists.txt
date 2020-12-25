cmake_minimum_required(VERSION 3.0.2)
project(sobit_education_library)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  geometry_msgs
  nav_msgs
  pybind11_catkin
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
                            src/sobit_education_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

pybind_add_module(sobit_education_module MODULE src/sobit_turtlebot_controller.cpp
                                                src/sobit_education_controller.cpp
                                                src/sobit_education_pybind.cpp)