cmake_minimum_required(VERSION 2.6)

PROJECT(SpatialObjectToImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(SpatialObjectToImageFilter SpatialObjectToImageFilter.cxx)
TARGET_LINK_LIBRARIES(SpatialObjectToImageFilter ITKIO)
