cmake_minimum_required(VERSION 2.6)

PROJECT(RungeKutta4)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RungeKutta4 RungeKutta4.cxx)

TARGET_LINK_LIBRARIES(RungeKutta4 vtkHybrid)
