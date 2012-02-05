cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(CustomPointType)
find_package(PCL 1.0 REQUIRED COMPONENTS io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(CustomPointType CustomPointType.cpp)
target_link_libraries (CustomPointType ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

add_executable(works works.cpp)
target_link_libraries (works ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
