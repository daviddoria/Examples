cmake_minimum_required(VERSION 2.6)

PROJECT(LabelVertices)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(LabelVertices LabelVertices.cxx)
TARGET_LINK_LIBRARIES(LabelVertices vtkHybrid  vtkViews)

