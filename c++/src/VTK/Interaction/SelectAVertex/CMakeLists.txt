cmake_minimum_required(VERSION 2.6)

PROJECT(SelectAVertex)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SelectAVertex SelectAVertex.cxx)

TARGET_LINK_LIBRARIES(SelectAVertex vtkHybrid)
