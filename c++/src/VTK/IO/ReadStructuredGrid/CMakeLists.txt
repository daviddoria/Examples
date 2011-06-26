cmake_minimum_required(VERSION 2.6)

PROJECT(ReadStructuredGrid)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ReadStructuredGrid ReadStructuredGrid.cxx)
TARGET_LINK_LIBRARIES(ReadStructuredGrid vtkHybrid)
