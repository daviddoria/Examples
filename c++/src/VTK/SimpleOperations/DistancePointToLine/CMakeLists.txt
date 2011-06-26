cmake_minimum_required(VERSION 2.6)

PROJECT(DistancePointToLine)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DistancePointToLine DistancePointToLine.cxx)

TARGET_LINK_LIBRARIES(DistancePointToLine vtkHybrid)
