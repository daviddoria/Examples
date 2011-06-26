cmake_minimum_required(VERSION 2.6)

PROJECT(VisualizeVTPFile)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(VisualizeVTPFile VisualizeVTPFile)
TARGET_LINK_LIBRARIES(VisualizeVTPFile vtkHybrid)

