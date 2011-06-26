cmake_minimum_required(VERSION 2.6)
PROJECT(CellCenters)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CellCenters CellCenters.cxx)
TARGET_LINK_LIBRARIES(CellCenters vtkHybrid)

