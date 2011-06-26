cmake_minimum_required(VERSION 2.6)

PROJECT(RubberBandPick)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RubberBandPick RubberBandPick.cxx)
TARGET_LINK_LIBRARIES(RubberBandPick vtkHybrid)
