cmake_minimum_required(VERSION 2.6)

PROJECT(SetMiscDataCells)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SetMiscDataCells SetMiscDataCells.cpp)
TARGET_LINK_LIBRARIES(SetMiscDataCells vtkHybrid)

