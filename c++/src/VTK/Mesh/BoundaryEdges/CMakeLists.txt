cmake_minimum_required(VERSION 2.6)

PROJECT(BoundaryEdges)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BoundaryEdges BoundaryEdges.cxx)
TARGET_LINK_LIBRARIES(BoundaryEdges vtkHybrid)

