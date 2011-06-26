cmake_minimum_required(VERSION 2.6)

PROJECT(RotateActor)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RotateActor RotateActor.cxx)

TARGET_LINK_LIBRARIES(RotateActor vtkHybrid)
