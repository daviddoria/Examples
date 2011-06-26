cmake_minimum_required(VERSION 2.6)

PROJECT(MultipleActors)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MultipleActors MultipleActors.cxx)
TARGET_LINK_LIBRARIES(MultipleActors vtkHybrid)
