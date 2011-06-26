cmake_minimum_required(VERSION 2.6)
PROJECT(ImageMagnitude)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageMagnitude ImageMagnitude.cxx)
TARGET_LINK_LIBRARIES(ImageMagnitude vtkHybrid)
