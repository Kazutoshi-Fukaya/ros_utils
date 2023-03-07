cmake_minimum_required(VERSION 3.0.2)
project(objects_data_publisher)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  multi_localizer_msgs
  object_detector_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# objects_data_publisher_node
add_executable(objects_data_publisher_node
    src/objects_data_publisher_node.cpp
    src/objects_data_publisher.cpp
)
target_link_libraries(objects_data_publisher_node ${catkin_LIBRARIES})
