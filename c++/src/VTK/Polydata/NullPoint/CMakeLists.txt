cmake_minimum_required(VERSION 2.6)

PROJECT(NullPoint)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(NullPoint NullPoint.cxx)
TARGET_LINK_LIBRARIES(NullPoint vtkHybrid)
