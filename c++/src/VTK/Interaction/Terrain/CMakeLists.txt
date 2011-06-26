cmake_minimum_required(VERSION 2.6)

PROJECT(Terrain)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Terrain Terrain.cxx)
TARGET_LINK_LIBRARIES(Terrain vtkHybrid)
