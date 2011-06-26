cmake_minimum_required(VERSION 2.6)

PROJECT(InteractWithVertices)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(InteractWithVertices InteractWithVertices.cxx)
TARGET_LINK_LIBRARIES(InteractWithVertices vtkHybrid  vtkViews)

