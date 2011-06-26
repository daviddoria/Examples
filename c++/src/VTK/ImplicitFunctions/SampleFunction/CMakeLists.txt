cmake_minimum_required(VERSION 2.6)

PROJECT(SampleFunction)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SampleFunction SampleFunction.cxx)
TARGET_LINK_LIBRARIES(SampleFunction vtkHybrid)

