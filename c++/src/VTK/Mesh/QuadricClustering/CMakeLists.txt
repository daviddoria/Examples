cmake_minimum_required(VERSION 2.6)

PROJECT(QuadricClustering)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(QuadricClustering QuadricClustering.cxx)
TARGET_LINK_LIBRARIES(QuadricClustering vtkHybrid)
