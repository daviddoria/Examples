cmake_minimum_required(VERSION 2.6)
PROJECT(StructuredGridVisualizeCells)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(StructuredGridVisualizeCells StructuredGridVisualizeCells.cxx)
TARGET_LINK_LIBRARIES(StructuredGridVisualizeCells vtkHybrid)

