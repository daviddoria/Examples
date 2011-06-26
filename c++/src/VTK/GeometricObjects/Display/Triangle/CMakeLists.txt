cmake_minimum_required(VERSION 2.6)

PROJECT(Triangle)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DisplayTriangle DisplayTriangle.cxx)
TARGET_LINK_LIBRARIES(DisplayTriangle vtkHybrid)
