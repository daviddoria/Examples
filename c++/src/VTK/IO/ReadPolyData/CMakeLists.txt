cmake_minimum_required(VERSION 2.6)

PROJECT(ReadPolyData)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ReadPolyData ReadPolyData.cxx)
TARGET_LINK_LIBRARIES(ReadPolyData vtkHybrid)
