cmake_minimum_required(VERSION 2.6)

PROJECT(ScalarToRGBColormapImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ScalarToRGBColormapImageFilter ScalarToRGBColormapImageFilter.cxx)
TARGET_LINK_LIBRARIES(ScalarToRGBColormapImageFilter ITKIO)
