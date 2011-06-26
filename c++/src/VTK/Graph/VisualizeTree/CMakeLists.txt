cmake_minimum_required(VERSION 2.6)

PROJECT(VisualizeTree)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(VisualizeTree VisualizeTree.cxx)
TARGET_LINK_LIBRARIES(VisualizeTree vtkHybrid  vtkViews)

