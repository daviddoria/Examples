cmake_minimum_required(VERSION 2.6)

PROJECT(CellPicking)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CellPicking CellPicking.cxx)
TARGET_LINK_LIBRARIES(CellPicking vtkHybrid)
