cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(NormalEstimationWithCurvature)

find_package(PCL 1.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(NormalEstimationWithCurvature NormalEstimationWithCurvature.cpp)
target_link_libraries (NormalEstimationWithCurvature ${PCL_LIBRARIES})
