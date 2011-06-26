cmake_minimum_required(VERSION 2.6)

PROJECT(ImageReslice)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageReslice ImageReslice.cxx)
TARGET_LINK_LIBRARIES(ImageReslice vtkHybrid)

