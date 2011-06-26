cmake_minimum_required(VERSION 2.6)

PROJECT(HighlightPoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(HighlightPoints HighlightPoints.cxx)
TARGET_LINK_LIBRARIES(HighlightPoints vtkHybrid)
