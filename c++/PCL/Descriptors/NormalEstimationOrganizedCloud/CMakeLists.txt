cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(NormalEstimationOrganizedCloud)

find_package(PCL 1.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(NormalEstimationOrganizedCloud NormalEstimationOrganizedCloud.cpp)
target_link_libraries (NormalEstimationOrganizedCloud ${PCL_LIBRARIES})

