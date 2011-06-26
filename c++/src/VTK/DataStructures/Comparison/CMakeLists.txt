cmake_minimum_required(VERSION 2.6)

PROJECT(Comparison)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Comparison Comparison.cxx)
TARGET_LINK_LIBRARIES(Comparison vtkHybrid)

