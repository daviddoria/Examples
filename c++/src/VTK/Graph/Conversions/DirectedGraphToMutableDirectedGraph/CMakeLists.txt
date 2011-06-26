cmake_minimum_required(VERSION 2.6)

PROJECT(DirectedGraphToMutableDirectedGraph)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DirectedGraphToMutableDirectedGraph DirectedGraphToMutableDirectedGraph.cxx)
TARGET_LINK_LIBRARIES(DirectedGraphToMutableDirectedGraph vtkHybrid vtkInfovis)

