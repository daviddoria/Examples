cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(ClusteredViewpointFeatureHistogram)

#find_package(PCL 1.0 REQUIRED COMPONENTS io)
#find_package(PCL 1.2 REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ClusteredViewpointFeatureHistogram ClusteredViewpointFeatureHistogram.cpp)

target_link_libraries(ClusteredViewpointFeatureHistogram
${PCL_LIBRARIES}
)