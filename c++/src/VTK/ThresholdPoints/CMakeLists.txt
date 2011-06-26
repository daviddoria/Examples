cmake_minimum_required(VERSION 2.6)

PROJECT(ThresholdPoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ThresholdPoints ThresholdPoints.cxx)
TARGET_LINK_LIBRARIES(ThresholdPoints vtkHybrid)
