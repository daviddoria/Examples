cmake_minimum_required(VERSION 2.6)

PROJECT(InEdgeIterator)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(InEdgeIterator InEdgeIterator.cxx)
TARGET_LINK_LIBRARIES(InEdgeIterator vtkHybrid vtkInfovis)

