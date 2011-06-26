cmake_minimum_required(VERSION 2.6)
PROJECT(MergeSelection)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MergeSelection MergeSelection.cxx)
TARGET_LINK_LIBRARIES(MergeSelection vtkHybrid)

