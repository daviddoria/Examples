cmake_minimum_required(VERSION 2.6)
PROJECT(ImageMirrorPad)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageMirrorPad ImageMirrorPad.cxx)
TARGET_LINK_LIBRARIES(ImageMirrorPad vtkHybrid)

