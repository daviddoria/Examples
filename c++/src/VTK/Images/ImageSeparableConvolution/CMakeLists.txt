cmake_minimum_required(VERSION 2.6)

PROJECT(ImageSeparableConvolution2D)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageSeparableConvolution ImageSeparableConvolution.cxx)
TARGET_LINK_LIBRARIES(ImageSeparableConvolution vtkHybrid)

