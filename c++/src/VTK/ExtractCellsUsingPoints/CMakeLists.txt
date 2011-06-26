cmake_minimum_required(VERSION 2.6)
PROJECT(ExtractCellsUsingPoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ExtractCellsUsingPoints ExtractCellsUsingPoints.cxx)
TARGET_LINK_LIBRARIES(ExtractCellsUsingPoints vtkHybrid)

