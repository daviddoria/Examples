cmake_minimum_required(VERSION 2.6)

PROJECT(HandleKeypress)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(HandleKeypress HandleKeypress.cxx)
TARGET_LINK_LIBRARIES(HandleKeypress vtkHybrid)
