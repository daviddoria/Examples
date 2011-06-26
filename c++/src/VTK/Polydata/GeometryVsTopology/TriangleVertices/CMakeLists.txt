cmake_minimum_required(VERSION 2.6)

PROJECT(TriangleVertices)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TriangleVertices TriangleVertices.cxx)
TARGET_LINK_LIBRARIES(TriangleVertices vtkHybrid)
