cmake_minimum_required(VERSION 2.6)

PROJECT(CursorShape)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CursorShape CursorShape.cxx)

TARGET_LINK_LIBRARIES(CursorShape vtkHybrid)
