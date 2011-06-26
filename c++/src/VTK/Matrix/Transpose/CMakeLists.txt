cmake_minimum_required(VERSION 2.6)
PROJECT(Transpose)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Transpose Transpose.cxx)
TARGET_LINK_LIBRARIES(Transpose vtkHybrid)

