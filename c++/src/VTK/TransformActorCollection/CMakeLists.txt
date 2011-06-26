cmake_minimum_required(VERSION 2.6)

PROJECT(TransformActorCollection)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TransformActorCollection TransformActorCollection.cxx)
TARGET_LINK_LIBRARIES(TransformActorCollection vtkHybrid)
