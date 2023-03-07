cmake_minimum_required(VERSION 3.0.2)
project(images_connector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

# OpenCV
find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 3.0 QUIET)
    if(NOT OpenCV_FOUND)
        find_package(OpenCV 3.4.3 QUIET)
        if(NOT OpenCV_FOUND)
            message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
        endif()
    endif()
endif()

catkin_package(
    INCLUDE_DIRS include
    ${OpenCV_INCLUDE_DIRS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# images_connector_node
add_executable(${PROJECT_NAME}_node 
    src/image_subscriber.cpp
    src/images_connector.cpp
    src/images_connector_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)