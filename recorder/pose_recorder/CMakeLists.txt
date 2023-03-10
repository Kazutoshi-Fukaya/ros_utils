cmake_minimum_required(VERSION 3.0.2)
project(pose_recorder)

add_compile_options(-o2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# pose_recorder_node
add_executable(pose_recorder_node
    src/pose_recorder.cpp
    src/pose_recorder_node.cpp
)
target_link_libraries(pose_recorder_node ${catkin_LIBRARIES})
