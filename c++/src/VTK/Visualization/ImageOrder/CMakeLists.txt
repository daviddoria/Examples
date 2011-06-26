cmake_minimum_required(VERSION 2.6)

PROJECT(ImageOrder)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageOrder ImageOrder.cxx)
TARGET_LINK_LIBRARIES(ImageOrder vtkHybrid)

