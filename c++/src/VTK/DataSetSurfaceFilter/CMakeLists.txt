cmake_minimum_required(VERSION 2.6)

PROJECT(DataSetSurfaceFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DataSetSurfaceFilter DataSetSurfaceFilter.cxx)
TARGET_LINK_LIBRARIES(DataSetSurfaceFilter vtkHybrid)
