cmake_minimum_required(VERSION 2.6)

PROJECT(DoubleClick)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DoubleClick DoubleClick.cxx)
TARGET_LINK_LIBRARIES(DoubleClick vtkHybrid)

