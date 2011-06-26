cmake_minimum_required(VERSION 2.6)
PROJECT(QuantizePolyDataPoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(QuantizePolyDataPoints QuantizePolyDataPoints.cxx)
TARGET_LINK_LIBRARIES(QuantizePolyDataPoints vtkHybrid)

