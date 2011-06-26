cmake_minimum_required(VERSION 2.6)

PROJECT(ScaleVertices)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ScaleVertices ScaleVertices.cxx)
TARGET_LINK_LIBRARIES(ScaleVertices vtkHybrid  vtkViews)

