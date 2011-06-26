cmake_minimum_required(VERSION 2.6)

PROJECT(Gradient)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Gradient Gradient.cxx)
TARGET_LINK_LIBRARIES(Gradient vtkHybrid)

