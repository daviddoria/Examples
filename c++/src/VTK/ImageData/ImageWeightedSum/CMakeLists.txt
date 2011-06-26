cmake_minimum_required(VERSION 2.6)
PROJECT(ImageWeightedSum)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageWeightedSum ImageWeightedSum.cxx)
TARGET_LINK_LIBRARIES(ImageWeightedSum vtkHybrid)

