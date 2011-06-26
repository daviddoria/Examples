cmake_minimum_required(VERSION 2.6)

PROJECT(ClosestPoint)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ClosestPoint ClosestPoint.cxx)
TARGET_LINK_LIBRARIES(ClosestPoint vtkHybrid)

