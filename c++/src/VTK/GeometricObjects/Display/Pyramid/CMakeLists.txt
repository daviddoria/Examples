cmake_minimum_required(VERSION 2.6)

PROJECT(Pyramid)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Pyramid Pyramid.cxx)

TARGET_LINK_LIBRARIES(Pyramid vtkHybrid)
