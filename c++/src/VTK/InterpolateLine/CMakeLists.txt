cmake_minimum_required(VERSION 2.6)

PROJECT(InterpolateLine)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(InterpolateLine InterpolateLine.cxx)
TARGET_LINK_LIBRARIES(InterpolateLine vtkHybrid)
