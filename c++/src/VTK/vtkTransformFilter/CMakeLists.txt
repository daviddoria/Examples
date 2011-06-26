cmake_minimum_required(VERSION 2.6)

PROJECT(TransformFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TransformFilter TransformFilter.cxx)
TARGET_LINK_LIBRARIES(TransformFilter vtkHybrid)
