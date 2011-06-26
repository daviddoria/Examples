cmake_minimum_required(VERSION 2.6)

PROJECT(SideBySideGraphs)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SideBySideGraphs SideBySideGraphs.cxx)
TARGET_LINK_LIBRARIES(SideBySideGraphs vtkHybrid vtkInfovis vtkViews)

