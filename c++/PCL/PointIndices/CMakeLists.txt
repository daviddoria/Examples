cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(PointIndices)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(PointIndices PointIndices.cpp)
target_link_libraries(PointIndices ${PCL_LIBRARIES})
