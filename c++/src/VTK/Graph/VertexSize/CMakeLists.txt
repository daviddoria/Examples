cmake_minimum_required(VERSION 2.6)

PROJECT(VertexSize)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(VertexSize VertexSize.cxx)
TARGET_LINK_LIBRARIES(VertexSize vtkHybrid  vtkViews)

