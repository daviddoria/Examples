cmake_minimum_required(VERSION 2.6)
PROJECT(ImageShrink3D)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageShrink3D ImageShrink3D.cxx)
TARGET_LINK_LIBRARIES(ImageShrink3D vtkHybrid)

