cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(PointCloud)

#find_package(PCL 1.0 REQUIRED COMPONENTS io)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(PointCloud PointCloud.cpp)
#target_link_libraries(PointCloud ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
target_link_libraries(PointCloud ${PCL_LIBRARIES})
