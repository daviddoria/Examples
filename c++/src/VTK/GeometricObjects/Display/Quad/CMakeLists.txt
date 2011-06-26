cmake_minimum_required(VERSION 2.6)

PROJECT(DisplayQuad)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DisplayQuad DisplayQuad.cxx)
TARGET_LINK_LIBRARIES(DisplayQuad vtkHybrid)

ADD_EXECUTABLE(QuadBad QuadBad.cxx)
TARGET_LINK_LIBRARIES(QuadBad vtkHybrid)

