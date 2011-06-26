cmake_minimum_required(VERSION 2.6)

PROJECT(ActorCollection)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ActorCollection ActorCollection.cxx)
TARGET_LINK_LIBRARIES(ActorCollection vtkHybrid)

