cmake_minimum_required(VERSION 2.6)

PROJECT(TableToGraph)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TableToGraph TableToGraph.cxx)
TARGET_LINK_LIBRARIES(TableToGraph vtkHybrid vtkViews)
