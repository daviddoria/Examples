cmake_minimum_required(VERSION 2.6)

PROJECT(ImageDilateErode3D3D)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageDilateErode3D ImageDilateErode3D.cxx)
TARGET_LINK_LIBRARIES(ImageDilateErode3D vtkHybrid)

