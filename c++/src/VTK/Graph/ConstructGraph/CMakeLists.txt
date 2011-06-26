cmake_minimum_required(VERSION 2.6)

PROJECT(ConstructGraph)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ConstructGraph ConstructGraph.cpp)
TARGET_LINK_LIBRARIES(ConstructGraph vtkHybrid)

