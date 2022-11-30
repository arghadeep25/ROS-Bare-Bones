cmake_minimum_required(VERSION 3.11)
project(ros_bare_bones)

set(CMAKE_CXX_STANDARD 14)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

find_package(catkin REQUIRED COMPONENTS nodelet roscpp)

catkin_package(INCLUDE_DIRS include)
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})


add_library(ros_bare_bones_nodelet apps/ros_barebones_nodelet.cpp)
target_link_libraries(ros_bare_bones_nodelet PUBLIC ${catkin_LIBRARIES})