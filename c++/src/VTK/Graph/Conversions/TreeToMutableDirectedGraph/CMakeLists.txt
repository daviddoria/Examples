cmake_minimum_required(VERSION 2.6)

PROJECT(TreeToMutableDirectedGraph)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TreeToMutableDirectedGraph TreeToMutableDirectedGraph.cxx)
TARGET_LINK_LIBRARIES(TreeToMutableDirectedGraph vtkHybrid vtkInfovis)

