cmake_minimum_required(VERSION 2.6)

PROJECT(Checkerboard)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Checkerboard Checkerboard.cxx)
TARGET_LINK_LIBRARIES(Checkerboard vtkHybrid)

