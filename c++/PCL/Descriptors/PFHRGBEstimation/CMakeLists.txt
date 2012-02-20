cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(PFHRGBEstimation)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(PFHRGBEstimation PFHRGBEstimation.cpp)

target_link_libraries(PFHRGBEstimation ${PCL_LIBRARIES})