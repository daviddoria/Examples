cmake_minimum_required(VERSION 2.6)

PROJECT(DisplayCube)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DisplayCube DisplayCube.cxx)

TARGET_LINK_LIBRARIES(DisplayCube vtkHybrid)
