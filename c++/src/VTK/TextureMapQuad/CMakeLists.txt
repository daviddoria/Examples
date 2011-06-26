cmake_minimum_required(VERSION 2.6)

PROJECT(TextureMapQuad)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TextureMapQuad TextureMapQuad.cxx)
TARGET_LINK_LIBRARIES(TextureMapQuad vtkHybrid)
