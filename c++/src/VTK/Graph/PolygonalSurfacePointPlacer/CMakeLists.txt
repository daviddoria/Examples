cmake_minimum_required(VERSION 2.6)

PROJECT(DijkstraGraphGeodesicPath)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DijkstraGraphGeodesicPath DijkstraGraphGeodesicPath.cxx)
TARGET_LINK_LIBRARIES(DijkstraGraphGeodesicPath vtkHybrid vtkInfovis)

