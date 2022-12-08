cmake_minimum_required(VERSION 3.10)
project(ros_bare_bones)

set(CMAKE_CXX_STANDARD 14)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

find_package(catkin REQUIRED COMPONENTS
        nodelet
        pcl_ros
        roscpp)

# PCL Libraries
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

catkin_package(INCLUDE_DIRS include)
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})


add_library(ros_bare_bones_nodelet apps/ros_barebones_nodelet.cpp)
target_link_libraries(ros_bare_bones_nodelet PUBLIC ${catkin_LIBRARIES} PUBLIC ${PCL_LIBRARIES})