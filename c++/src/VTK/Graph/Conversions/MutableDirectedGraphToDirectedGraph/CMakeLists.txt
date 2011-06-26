cmake_minimum_required(VERSION 2.6)

PROJECT(MutableDirectedGraphToDirectedGraph)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MutableDirectedGraphToDirectedGraph MutableDirectedGraphToDirectedGraph.cxx)
TARGET_LINK_LIBRARIES(MutableDirectedGraphToDirectedGraph vtkHybrid vtkInfovis)

