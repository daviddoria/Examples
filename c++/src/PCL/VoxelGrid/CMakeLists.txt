cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(VoxelGrid)

find_package(PCL 1.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(VoxelGrid VoxelGrid.cpp)
target_link_libraries (VoxelGrid ${PCL_LIBRARIES})

