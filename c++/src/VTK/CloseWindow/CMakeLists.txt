cmake_minimum_required(VERSION 2.6)
PROJECT(CloseWindow)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CloseWindow CloseWindow.cxx)
TARGET_LINK_LIBRARIES(CloseWindow vtkHybrid)

