cmake_minimum_required(VERSION 2.6)

PROJECT(MoveCamera)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MoveCamera MoveCamera.cxx)
TARGET_LINK_LIBRARIES(MoveCamera vtkHybrid)
