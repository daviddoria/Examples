cmake_minimum_required(VERSION 2.6)

PROJECT(ActorTransform)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ActorTransform ActorTransform.cxx)
TARGET_LINK_LIBRARIES(ActorTransform vtkHybrid)
