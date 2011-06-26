cmake_minimum_required(VERSION 2.6)

PROJECT(ImageSinusoidSource)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageSinusoidSource ImageSinusoidSource.cxx)
TARGET_LINK_LIBRARIES(ImageSinusoidSource vtkHybrid)
