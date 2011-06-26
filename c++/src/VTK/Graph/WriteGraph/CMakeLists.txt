cmake_minimum_required(VERSION 2.6)

PROJECT(WriteGraph)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(WriteGraph WriteGraph.cxx)
TARGET_LINK_LIBRARIES(WriteGraph vtkHybrid)

