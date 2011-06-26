cmake_minimum_required(VERSION 2.6)
PROJECT(ImageMandelbrotSource)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageMandelbrotSource ImageMandelbrotSource.cxx)
TARGET_LINK_LIBRARIES(ImageMandelbrotSource vtkHybrid)

