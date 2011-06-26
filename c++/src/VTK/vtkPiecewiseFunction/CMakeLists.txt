cmake_minimum_required(VERSION 2.6)
PROJECT(PiecewiseFunction)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PiecewiseFunction PiecewiseFunction.cxx)
TARGET_LINK_LIBRARIES(PiecewiseFunction vtkHybrid)

