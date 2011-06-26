cmake_minimum_required(VERSION 2.6)
PROJECT(ImageGradient)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageGradient ImageGradient.cxx)
TARGET_LINK_LIBRARIES(ImageGradient vtkHybrid)

