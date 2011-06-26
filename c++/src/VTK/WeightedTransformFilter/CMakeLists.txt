cmake_minimum_required(VERSION 2.6)

PROJECT(WeightedTransformFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(WeightedTransformFilter WeightedTransformFilter.cxx)
TARGET_LINK_LIBRARIES(WeightedTransformFilter vtkHybrid)
