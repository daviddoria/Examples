cmake_minimum_required(VERSION 2.6)

PROJECT(SmoothingRecursiveGaussianImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(SmoothingRecursiveGaussianImageFilter SmoothingRecursiveGaussianImageFilter.cxx)
TARGET_LINK_LIBRARIES(SmoothingRecursiveGaussianImageFilter ITKIO)

