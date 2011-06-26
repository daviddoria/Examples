cmake_minimum_required(VERSION 2.6)

PROJECT(NodeShape)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(NodeShape NodeShape.cxx)
TARGET_LINK_LIBRARIES(NodeShape vtkHybrid  vtkViews)

