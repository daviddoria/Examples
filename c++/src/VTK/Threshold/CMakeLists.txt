cmake_minimum_required(VERSION 2.6)

PROJECT(Threshold)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Threshold Threshold.cxx)
TARGET_LINK_LIBRARIES(Threshold vtkHybrid)
