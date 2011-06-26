cmake_minimum_required(VERSION 2.6)

PROJECT(DepthFirstSearch)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DepthFirstSearch DepthFirstSearch.cxx)
TARGET_LINK_LIBRARIES(DepthFirstSearch vtkHybrid vtkInfovis)
