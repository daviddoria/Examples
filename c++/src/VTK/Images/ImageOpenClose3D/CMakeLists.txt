cmake_minimum_required(VERSION 2.6)

PROJECT(ImageOpenClose3D)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageOpenClose3D ImageOpenClose3D.cxx)
TARGET_LINK_LIBRARIES(ImageOpenClose3D vtkHybrid)

