cmake_minimum_required(VERSION 2.6)

PROJECT(VectorDot)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(VectorDot VectorDot.cxx)
TARGET_LINK_LIBRARIES(VectorDot vtkHybrid)
