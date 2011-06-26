cmake_minimum_required(VERSION 2.6)

PROJECT(ReadGraph)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ReadGraph ReadGraph.cxx)
TARGET_LINK_LIBRARIES(ReadGraph vtkHybrid)

