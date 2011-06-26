cmake_minimum_required(VERSION 2.6)

PROJECT(ColoredLines)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ColoredLines ColoredLines.cxx)
TARGET_LINK_LIBRARIES(ColoredLines vtkHybrid)
