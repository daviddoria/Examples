cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(ConcatenateFields)

find_package(PCL 1.4 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ConcatenateFields ConcatenateFields.cpp)

target_link_libraries(ConcatenateFields ${PCL_LIBRARIES})
