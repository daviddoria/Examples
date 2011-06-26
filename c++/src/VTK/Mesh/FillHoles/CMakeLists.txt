cmake_minimum_required(VERSION 2.6)

PROJECT(FillHoles)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(FillHoles FillHoles.cxx)
TARGET_LINK_LIBRARIES(FillHoles vtkHybrid)

