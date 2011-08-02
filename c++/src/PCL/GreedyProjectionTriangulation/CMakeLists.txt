cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(GreedyProjectionTriangulation)

find_package(PCL 1.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(GreedyProjectionTriangulation GreedyProjectionTriangulation.cpp)
target_link_libraries(GreedyProjectionTriangulation ${PCL_LIBRARIES})
