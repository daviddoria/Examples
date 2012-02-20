cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(PointFeatureHistogram)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(PointFeatureHistogram PointFeatureHistogram.cpp)
target_link_libraries(PointFeatureHistogram ${PCL_LIBRARIES})

add_executable(SinglePoint SinglePoint.cpp)
target_link_libraries(SinglePoint ${PCL_LIBRARIES})

add_executable(Benchmarking Benchmarking.cpp)
target_link_libraries(Benchmarking ${PCL_LIBRARIES})
