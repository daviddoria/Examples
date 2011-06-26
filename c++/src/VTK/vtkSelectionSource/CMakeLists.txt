cmake_minimum_required(VERSION 2.6)
PROJECT(SelectionSource)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SelectionSource SelectionSource.cxx)
TARGET_LINK_LIBRARIES(SelectionSource vtkHybrid)

