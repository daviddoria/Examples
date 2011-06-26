cmake_minimum_required(VERSION 2.6)

PROJECT(ImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImageFilter ImageFilterExample.cxx)
TARGET_LINK_LIBRARIES(ImageFilter ITKBasicFilters ITKIO ITKCommon)
