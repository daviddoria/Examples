cmake_minimum_required(VERSION 2.6)

PROJECT(ReadUnstructuredGrid)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ReadUnstructuredGrid ReadUnstructuredGrid.cxx)
TARGET_LINK_LIBRARIES(ReadUnstructuredGrid vtkHybrid)
