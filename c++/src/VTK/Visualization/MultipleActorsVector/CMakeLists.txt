cmake_minimum_required(VERSION 2.6)

PROJECT(MultipleActorsVector)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MultipleActorsVector MultipleActorsVector.cxx)
TARGET_LINK_LIBRARIES(MultipleActorsVector vtkHybrid)
