cmake_minimum_required(VERSION 2.6)
PROJECT(StructuredPointsToUnstructuredGrid)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(StructuredPointsToUnstructuredGrid StructuredPointsToUnstructuredGrid.cxx)
TARGET_LINK_LIBRARIES(StructuredPointsToUnstructuredGrid vtkHybrid)

