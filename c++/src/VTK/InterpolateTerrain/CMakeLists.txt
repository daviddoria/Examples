cmake_minimum_required(VERSION 2.6)

PROJECT(InterpolateTerrain)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(InterpolateTerrain InterpolateTerrain.cxx)
TARGET_LINK_LIBRARIES(InterpolateTerrain vtkHybrid)
