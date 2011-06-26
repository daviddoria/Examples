cmake_minimum_required(VERSION 2.6)

PROJECT(SimpleElevationFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SimpleElevationFilter SimpleElevationFilter.cxx)
TARGET_LINK_LIBRARIES(SimpleElevationFilter vtkHybrid)

