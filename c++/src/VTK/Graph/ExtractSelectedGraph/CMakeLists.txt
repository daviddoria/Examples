cmake_minimum_required(VERSION 2.6)

PROJECT(ExtractSelectedGraph)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ExtractSelectedGraph ExtractSelectedGraph.cxx)
TARGET_LINK_LIBRARIES(ExtractSelectedGraph vtkHybrid vtkInfovis vtkViews)

