cmake_minimum_required(VERSION 2.6)

PROJECT(ClickRelease)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ClickRelease ClickRelease.cxx)
TARGET_LINK_LIBRARIES(ClickRelease vtkHybrid)

