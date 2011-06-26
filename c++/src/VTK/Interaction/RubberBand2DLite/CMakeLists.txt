cmake_minimum_required(VERSION 2.6)

PROJECT(RubberBand2DLite)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RubberBand2DLite RubberBand2DLite.cxx)
TARGET_LINK_LIBRARIES(RubberBand2DLite vtkHybrid)

