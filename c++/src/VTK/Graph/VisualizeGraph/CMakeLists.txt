cmake_minimum_required(VERSION 2.6)

PROJECT(VisualizeGraph)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(VisualizeGraph VisualizeGraph.cxx)
TARGET_LINK_LIBRARIES(VisualizeGraph vtkHybrid  vtkViews)

