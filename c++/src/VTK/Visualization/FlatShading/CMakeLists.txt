cmake_minimum_required(VERSION 2.6)

PROJECT(FlatShading)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(FlatShading FlatShading.cxx)
TARGET_LINK_LIBRARIES(FlatShading vtkHybrid)

