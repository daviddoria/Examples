cmake_minimum_required(VERSION 2.6)

PROJECT(ActorRepresentation)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ActorRepresentation ActorRepresentation.cxx)
TARGET_LINK_LIBRARIES(ActorRepresentation vtkHybrid)