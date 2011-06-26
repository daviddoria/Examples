cmake_minimum_required(VERSION 2.6)
PROJECT(ImageConvolve)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageConvolve ImageConvolve.cxx)
TARGET_LINK_LIBRARIES(ImageConvolve vtkHybrid)

