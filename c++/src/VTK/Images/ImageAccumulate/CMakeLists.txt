cmake_minimum_required(VERSION 2.6)

PROJECT(ImageAccumulate)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageAccumulate ImageAccumulate.cxx)
TARGET_LINK_LIBRARIES(ImageAccumulate vtkHybrid)
