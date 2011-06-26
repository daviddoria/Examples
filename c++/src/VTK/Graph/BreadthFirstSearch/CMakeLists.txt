cmake_minimum_required(VERSION 2.6)

PROJECT(BreadthFirstSearch)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BreadthFirstSearch BreadthFirstSearch.cpp)
TARGET_LINK_LIBRARIES(BreadthFirstSearch vtkHybrid vtkInfovis)
