cmake_minimum_required(VERSION 2.6)

PROJECT(ShiftClickActor)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ShiftClickActor ShiftClickActor.cxx)
TARGET_LINK_LIBRARIES(ShiftClickActor vtkHybrid)
