cmake_minimum_required(VERSION 2.6)

PROJECT(ImageFilterMultipleOutputs)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImageFilterMultipleOutputs ImageFilterMultipleOutputsExample.cxx)
TARGET_LINK_LIBRARIES(ImageFilterMultipleOutputs ITKBasicFilters ITKIO ITKCommon)
