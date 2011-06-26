cmake_minimum_required(VERSION 2.6)

PROJECT(TriangleLines)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TriangleLines TriangleLines.cxx)
TARGET_LINK_LIBRARIES(TriangleLines vtkHybrid)
