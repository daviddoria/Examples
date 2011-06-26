cmake_minimum_required(VERSION 2.6)

PROJECT(ShortestPath)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ShortestPath ShortestPath.cxx)
TARGET_LINK_LIBRARIES(ShortestPath vtkHybrid vtkInfovis vtkViews)
