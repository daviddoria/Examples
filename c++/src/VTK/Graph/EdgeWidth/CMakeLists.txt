cmake_minimum_required(VERSION 2.6)

PROJECT(EdgeWidth)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(EdgeWidth EdgeWidth.cxx)
TARGET_LINK_LIBRARIES(EdgeWidth vtkHybrid  vtkViews)

