cmake_minimum_required(VERSION 2.6)
PROJECT(StructuredGridVisualizePoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(StructuredGridVisualizePoints StructuredGridVisualizePoints.cxx)
TARGET_LINK_LIBRARIES(StructuredGridVisualizePoints vtkHybrid)

