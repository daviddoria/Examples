cmake_minimum_required(VERSION 2.6)

PROJECT(AdjacentVertexIterator)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(AdjacentVertexIteratorGraph AdjacentVertexIteratorGraph.cxx)
TARGET_LINK_LIBRARIES(AdjacentVertexIteratorGraph vtkHybrid vtkInfovis)

ADD_EXECUTABLE(AdjacentVertexIteratorTree AdjacentVertexIteratorTree.cxx)
TARGET_LINK_LIBRARIES(AdjacentVertexIteratorTree vtkHybrid vtkInfovis)

