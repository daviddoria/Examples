cmake_minimum_required(VERSION 2.6)

PROJECT(DragableOff)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DragableOff DragableOff.cxx)
TARGET_LINK_LIBRARIES(DragableOff vtkHybrid)
