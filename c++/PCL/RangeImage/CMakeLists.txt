cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(RangeImage)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(RangeImage RangeImage.cpp)
target_link_libraries(RangeImage ${PCL_LIBRARIES})
