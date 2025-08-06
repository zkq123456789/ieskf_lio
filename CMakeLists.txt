cmake_minimum_required(VERSION 3.0.2)
project(ieskf_slam)

## Compile as C++11, supported in ROS Kinetic and newer
SET( CMAKE_CXX_FLAGS "-std=c++17 -O3" )
add_compile_options(-std=c++17)
add_definitions(-DPROJ_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)
find_package(PCL REQUIRED)
# YAML CPP
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ieskf_slam
 CATKIN_DEPENDS roscpp rospy sensor_msgs
 DEPENDS system_lib
)

add_library(ieskf_core  # 库名称：ieskf_core
  src/ieskf_slam/frontEnd.cpp
)
target_link_libraries(ieskf_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)
# SET(IESKF_SLAM_FILE
#     src/ieskf_slam/frontEnd.cpp
# )
SET(NOETIC_FRONTEND_WRAPPER_FILE
    src/ros_include/ieskfFrontEndWrapper.cpp
)
add_executable(ieskf_frontend_node src/ieskf_running_node.cpp
${NOETIC_FRONTEND_WRAPPER_FILE}
)

target_link_libraries(ieskf_frontend_node
  ieskf_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
