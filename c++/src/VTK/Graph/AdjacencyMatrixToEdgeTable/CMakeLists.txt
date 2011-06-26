cmake_minimum_required(VERSION 2.6)

PROJECT(AdjacencyMatrixToEdgeTable)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(AdjacencyMatrixToEdgeTable AdjacencyMatrixToEdgeTable.cxx)
TARGET_LINK_LIBRARIES(AdjacencyMatrixToEdgeTable vtkHybrid vtkCommon vtkInfovis vtkViews)
