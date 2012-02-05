cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(RemoveNaNFromPointCloud)
find_package(PCL 1.0 REQUIRED COMPONENTS io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(RemoveNaNFromPointCloud RemoveNaNFromPointCloud.cpp)

target_link_libraries(RemoveNaNFromPointCloud ${PCL_COMMON_LIBRARIES} pcl_filters)
