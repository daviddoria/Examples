cmake_minimum_required(VERSION 2.6)

PROJECT(TubeFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TubeFilter TubeFilter.cxx)
TARGET_LINK_LIBRARIES(TubeFilter vtkHybrid)

