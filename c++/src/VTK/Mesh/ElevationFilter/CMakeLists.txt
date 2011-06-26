cmake_minimum_required(VERSION 2.6)

PROJECT(ElevationFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ElevationFilter ElevationFilter.cxx)
TARGET_LINK_LIBRARIES(ElevationFilter vtkHybrid)

