cmake_minimum_required(VERSION 2.6)

PROJECT(TransformActor)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TransformActor TransformActor.cxx)

TARGET_LINK_LIBRARIES(TransformActor vtkHybrid)
