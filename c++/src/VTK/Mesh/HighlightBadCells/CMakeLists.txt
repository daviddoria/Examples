cmake_minimum_required(VERSION 2.6)

PROJECT(HighlightBadCells)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(HighlightBadCells HighlightBadCells.cxx)
TARGET_LINK_LIBRARIES(HighlightBadCells vtkHybrid)

