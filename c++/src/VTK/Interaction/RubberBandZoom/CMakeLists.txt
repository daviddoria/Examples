cmake_minimum_required(VERSION 2.6)

PROJECT(RubberBandZoom)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RubberBandZoom RubberBandZoom.cxx)
TARGET_LINK_LIBRARIES(RubberBandZoom vtkHybrid)
