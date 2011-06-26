cmake_minimum_required(VERSION 2.6)

PROJECT(SelectAnActor)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SelectAnActor SelectAnActor.cxx)
TARGET_LINK_LIBRARIES(SelectAnActor vtkHybrid)
