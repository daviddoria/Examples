cmake_minimum_required(VERSION 2.6)

PROJECT(ColoredElevationMap)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ColoredElevationMap ColoredElevationMap.cxx)
TARGET_LINK_LIBRARIES(ColoredElevationMap vtkHybrid)

