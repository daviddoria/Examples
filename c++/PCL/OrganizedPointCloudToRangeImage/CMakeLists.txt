cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(OrganizedPointCloudToRangeImage)

find_package(PCL 1.5 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(OrganizedPointCloudToRangeImage OrganizedPointCloudToRangeImage.cpp)
target_link_libraries(OrganizedPointCloudToRangeImage ${PCL_LIBRARIES})

add_executable(OrganizedPointCloudToRangeImageManual OrganizedPointCloudToRangeImageManual.cpp)
target_link_libraries(OrganizedPointCloudToRangeImageManual ${PCL_LIBRARIES})
