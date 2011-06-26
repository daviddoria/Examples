cmake_minimum_required(VERSION 2.6)

PROJECT(GetMiscDataFromCells)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

######################################
ADD_EXECUTABLE(GetMiscDataFromCells GetMiscDataFromCells.cpp)
TARGET_LINK_LIBRARIES(GetMiscDataFromCells vtkHybrid)

