cmake_minimum_required(VERSION 2.6)

PROJECT(ImageMagnify)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageMagnify ImageMagnify.cxx)
TARGET_LINK_LIBRARIES(ImageMagnify vtkHybrid)

