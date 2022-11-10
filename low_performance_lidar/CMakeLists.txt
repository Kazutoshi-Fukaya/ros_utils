cmake_minimum_required(VERSION 3.0.2)
project(low_performance_lidar)

add_compile_options(-O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
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

# low_performance_lidar_node
add_executable(low_performance_lidar_node
    src/low_performance_lidar.cpp
    src/low_performance_lidar_node.cpp
)
target_link_libraries(low_performance_lidar_node ${catkin_LIBRARIES})
