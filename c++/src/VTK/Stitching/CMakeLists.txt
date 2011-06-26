cmake_minimum_required(VERSION 2.6)

PROJECT(Stitching)

set(CMAKE_BUILD_TYPE Debug)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

############
ADD_EXECUTABLE(Stitching Stitching.cpp)
TARGET_LINK_LIBRARIES(Stitching vtkHybrid)

