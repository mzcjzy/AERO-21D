cmake_minimum_required(VERSION 2.8.3)
project(way_follower)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(autoware_config_msgs REQUIRED)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        tf
        tf2
        geometry_msgs
        pcl_ros
        pcl_conversions
        sensor_msgs
        autoware_msgs
        autoware_config_msgs
        rostest
        rosunit
        )

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES libwaypoint_follower
        CATKIN_DEPENDS roscpp
        std_msgs
        tf
        tf2
        geometry_msgs
        autoware_msgs
        autoware_config_msgs
        amathutils_lib
        pcl_ros
        pcl_conversions
        sensor_msgs

        gazebo_msgs
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        include
        
        ${autoware_config_msgs_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
       
        ${catkin_INCLUDE_DIRS}
)

add_library(libwaypoint_follower lib/libwaypoint_follower.cpp)
add_dependencies(libwaypoint_follower
        ${catkin_EXPORTED_TARGETS}
        autoware_msgs_generate_messages_cpp
        )
target_link_libraries(libwaypoint_follower ${catkin_LIBRARIES})

add_executable(pure
        nodes/pure_pursuit/pure_pursuit_node.cpp
        nodes/pure_pursuit/pure_pursuit.cpp
        nodes/pure_pursuit/pure_pursuit_core.cpp
        nodes/pure_pursuit/pure_pursuit_viz.cpp
        )
target_link_libraries(pure libwaypoint_follower ${catkin_LIBRARIES})
add_dependencies(pure ${catkin_EXPORTED_TARGETS})












