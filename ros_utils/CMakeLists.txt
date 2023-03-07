cmake_minimum_required(VERSION 3.0.2)
project(ros_utils)

find_package(catkin REQUIRED COMPONENTS
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

# add_executable(${PROJECT_NAME}_node src/ros_utils_node.cpp)
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

