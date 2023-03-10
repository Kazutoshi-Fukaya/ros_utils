cmake_minimum_required(VERSION 3.0.2)
project(object_detection_recorder)

find_package(catkin REQUIRED COMPONENTS
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

# object_detection_recorder_node
add_executable(${PROJECT_NAME}_node
  src/recorded_objects.cpp
  src/object_detection_recorder.cpp
  src/object_detection_recorder_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
