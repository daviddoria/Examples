cmake_minimum_required(VERSION 2.6)

PROJECT(LaplacianRecursiveGaussianImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(LaplacianRecursiveGaussianImageFilter LaplacianRecursiveGaussianImageFilter.cxx)
TARGET_LINK_LIBRARIES(LaplacianRecursiveGaussianImageFilter ${ITK_LIBRARIES})
