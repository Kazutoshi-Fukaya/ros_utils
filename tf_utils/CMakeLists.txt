cmake_minimum_required(VERSION 3.0.2)
project(tf_utils)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
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

# rosbag_tf
add_executable(rosbag_tf_node
    src/rosbag_tf.cpp
    src/rosbag_tf_node.cpp
)
target_link_libraries(rosbag_tf_node ${catkin_LIBRARIES})


