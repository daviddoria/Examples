cmake_minimum_required(VERSION 2.6)

PROJECT(ReadAndDisplay3DGraph)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ReadAndDisplay3DGraph ReadAndDisplay3DGraph.cxx)
TARGET_LINK_LIBRARIES(ReadAndDisplay3DGraph vtkHybrid  vtkViews)

