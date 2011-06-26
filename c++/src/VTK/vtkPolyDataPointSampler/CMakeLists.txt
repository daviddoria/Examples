cmake_minimum_required(VERSION 2.6)
PROJECT(PolyDataPointSampler)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PolyDataPointSampler PolyDataPointSampler.cxx)
TARGET_LINK_LIBRARIES(PolyDataPointSampler vtkHybrid)

