cmake_minimum_required(VERSION 2.6)

PROJECT(GeometryFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(GeometryFilter GeometryFilter.cxx)
TARGET_LINK_LIBRARIES(GeometryFilter vtkHybrid)
