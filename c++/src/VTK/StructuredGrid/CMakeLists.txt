cmake_minimum_required(VERSION 2.6)
PROJECT(StructuredGrid)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(StructuredGrid StructuredGrid.cxx)
TARGET_LINK_LIBRARIES(StructuredGrid ${VTK_LIBRARIES})
