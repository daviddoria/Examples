cmake_minimum_required(VERSION 2.6)

PROJECT(TrackballCamera)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TrackballCamera TrackballCamera.cxx)
TARGET_LINK_LIBRARIES(TrackballCamera vtkHybrid)
