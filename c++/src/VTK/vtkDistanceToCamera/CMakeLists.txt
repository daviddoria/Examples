cmake_minimum_required(VERSION 2.6)

PROJECT(DistanceToCamera)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DistanceToCamera DistanceToCamera.cxx)
TARGET_LINK_LIBRARIES(DistanceToCamera vtkHybrid)
