cmake_minimum_required(VERSION 2.6)
PROJECT(StructuredGridBlanking)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(StructuredGridBlanking StructuredGridBlanking.cxx)
TARGET_LINK_LIBRARIES(StructuredGridBlanking vtkHybrid)
