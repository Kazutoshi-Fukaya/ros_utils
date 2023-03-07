cmake_minimum_required(VERSION 3.0.2)
project(laser_scan_to_point_cloud)

add_compile_options(-O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
    laser_geometry
    roscpp
    rospy
    sensor_msgs
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    ${catkin_INCLUDE_DIRS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# laser_scan_to_point_cloud_node
add_executable(laser_scan_to_point_cloud_node
    src/laser_scan_to_point_cloud.cpp
    src/laser_scan_to_point_cloud_node.cpp)
target_link_libraries(laser_scan_to_point_cloud_node ${catkin_LIBRARIES})