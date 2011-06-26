cmake_minimum_required(VERSION 2.6)
PROJECT(ImageGradientMagnitude)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageGradientMagnitude ImageGradientMagnitude.cxx)
TARGET_LINK_LIBRARIES(ImageGradientMagnitude vtkHybrid)

