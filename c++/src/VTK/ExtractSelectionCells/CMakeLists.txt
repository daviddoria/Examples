cmake_minimum_required(VERSION 2.6)
PROJECT(ExtractSelection)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ExtractSelectionCells ExtractSelectionCells.cxx)
TARGET_LINK_LIBRARIES(ExtractSelectionCells vtkHybrid)
