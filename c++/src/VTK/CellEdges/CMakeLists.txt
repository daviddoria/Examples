cmake_minimum_required(VERSION 2.6)

PROJECT(CellEdges)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CellEdges CellEdges.cxx)
TARGET_LINK_LIBRARIES(CellEdges vtkHybrid)

