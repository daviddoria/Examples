cmake_minimum_required(VERSION 2.6)

PROJECT(ParametricSpline)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ParametricSpline ParametricSpline.cxx)
TARGET_LINK_LIBRARIES(ParametricSpline vtkHybrid)

