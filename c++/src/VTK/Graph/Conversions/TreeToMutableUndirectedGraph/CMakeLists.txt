cmake_minimum_required(VERSION 2.6)

PROJECT(TreeToMutableUndirectedGraph)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TreeToMutableUndirectedGraph TreeToMutableUndirectedGraph.cxx)
TARGET_LINK_LIBRARIES(TreeToMutableUndirectedGraph vtkHybrid vtkInfovis)

