cmake_minimum_required(VERSION 2.6)

PROJECT(GradientFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(GradientFilter GradientFilter.cxx)
TARGET_LINK_LIBRARIES(GradientFilter vtkHybrid)
