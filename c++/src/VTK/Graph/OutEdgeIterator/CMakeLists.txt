cmake_minimum_required(VERSION 2.6)

PROJECT(OutEdgeIterator)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(OutEdgeIterator OutEdgeIterator.cxx)
TARGET_LINK_LIBRARIES(OutEdgeIterator vtkHybrid vtkInfovis)

