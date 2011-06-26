cmake_minimum_required(VERSION 2.6)

PROJECT(TriangulateTerrainMap)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TriangulateTerrainMap TriangulateTerrainMap.cxx)
TARGET_LINK_LIBRARIES(TriangulateTerrainMap vtkHybrid)
