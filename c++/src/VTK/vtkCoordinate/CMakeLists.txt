cmake_minimum_required(VERSION 2.6)
PROJECT(Coordinate)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Coordinate Coordinate.cxx)
TARGET_LINK_LIBRARIES(Coordinate vtkHybrid)

