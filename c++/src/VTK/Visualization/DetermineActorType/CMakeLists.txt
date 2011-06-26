cmake_minimum_required(VERSION 2.6)

PROJECT(DetermineType)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DetermineType DetermineType.cxx)
TARGET_LINK_LIBRARIES(DetermineType vtkHybrid)
