cmake_minimum_required(VERSION 2.6)

PROJECT(VisualizeDirectedGraph)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(VisualizeDirectedGraph VisualizeDirectedGraph.cxx)
TARGET_LINK_LIBRARIES(VisualizeDirectedGraph vtkHybrid  vtkViews)

