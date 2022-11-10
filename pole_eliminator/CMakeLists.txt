cmake_minimum_required(VERSION 3.0.2)
project(pole_eliminator)

add_compile_options(-std=c++11 -O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    std_msgs
)

catkin_package(INCLUDE_DIRS include)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# pole_eliminator_node
add_executable(pole_eliminator_node
    src/pole_eliminator_node.cpp
    src/pole_eliminator.cpp
)
target_link_libraries(pole_eliminator_node ${catkin_LIBRARIES})