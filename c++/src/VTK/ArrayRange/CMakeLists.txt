cmake_minimum_required(VERSION 2.6)

PROJECT(ArrayRange)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ArrayRange ArrayRange.cxx)
TARGET_LINK_LIBRARIES(ArrayRange vtkHybrid)
