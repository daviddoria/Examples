cmake_minimum_required(VERSION 2.6)

PROJECT(BackgroundColor)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BackgroundColor BackgroundColor.cxx)
TARGET_LINK_LIBRARIES(BackgroundColor vtkHybrid)
