cmake_minimum_required(VERSION 2.6)
PROJECT(RotateAroundLine)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RotateAroundLine RotateAroundLine.cxx)
TARGET_LINK_LIBRARIES(RotateAroundLine vtkHybrid)

