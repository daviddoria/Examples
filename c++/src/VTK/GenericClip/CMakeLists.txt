cmake_minimum_required(VERSION 2.6)

PROJECT(GenericClip)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(GenericClip GenericClip.cxx)
TARGET_LINK_LIBRARIES(GenericClip vtkHybrid vtkGenericFiltering)
