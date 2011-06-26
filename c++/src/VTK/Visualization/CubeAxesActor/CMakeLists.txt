cmake_minimum_required(VERSION 2.6)

PROJECT(CubeAxesActor)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CubeAxesActor CubeAxesActor.cxx)
TARGET_LINK_LIBRARIES(CubeAxesActor vtkHybrid)
