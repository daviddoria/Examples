cmake_minimum_required(VERSION 2.6)

PROJECT(WindowSize)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(WindowSize WindowSize.cxx)
TARGET_LINK_LIBRARIES(WindowSize vtkHybrid)
