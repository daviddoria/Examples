cmake_minimum_required(VERSION 2.6)

PROJECT(MoveActor)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MoveActor MoveActor.cxx)
TARGET_LINK_LIBRARIES(MoveActor vtkHybrid)
