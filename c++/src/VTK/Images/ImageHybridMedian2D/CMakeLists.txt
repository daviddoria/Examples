cmake_minimum_required(VERSION 2.6)

PROJECT(ImageHybridMedian2D)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageHybridMedian2D ImageHybridMedian2D.cxx)
TARGET_LINK_LIBRARIES(ImageHybridMedian2D vtkHybrid)

