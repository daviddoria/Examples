cmake_minimum_required(VERSION 2.6)

PROJECT(Outline)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Outline Outline.cxx)

TARGET_LINK_LIBRARIES(Outline vtkHybrid)
