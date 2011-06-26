cmake_minimum_required(VERSION 2.6)

PROJECT(GreedyTerrainDecimation)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(GreedyTerrainDecimation GreedyTerrainDecimation.cxx)

TARGET_LINK_LIBRARIES(GreedyTerrainDecimation vtkHybrid)
