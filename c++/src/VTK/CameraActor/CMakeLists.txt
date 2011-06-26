cmake_minimum_required(VERSION 2.6)

PROJECT(CameraActor)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CameraActor CameraActor.cxx)
TARGET_LINK_LIBRARIES(CameraActor vtkHybrid)
