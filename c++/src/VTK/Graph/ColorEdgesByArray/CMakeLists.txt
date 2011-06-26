cmake_minimum_required(VERSION 2.6)

PROJECT(ColorEdgesByArray)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ColorEdgesByArray ColorEdgesByArray.cxx)
TARGET_LINK_LIBRARIES(ColorEdgesByArray vtkHybrid vtkInfovis vtkViews)
