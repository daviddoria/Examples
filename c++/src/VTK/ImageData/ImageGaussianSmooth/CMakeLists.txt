cmake_minimum_required(VERSION 2.6)
PROJECT(ImageGaussianSmooth)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageGaussianSmooth ImageGaussianSmooth.cxx)
TARGET_LINK_LIBRARIES(ImageGaussianSmooth vtkHybrid)

