cmake_minimum_required(VERSION 2.6)

PROJECT(FixedArray)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(FixedArray FixedArray.cxx)
TARGET_LINK_LIBRARIES(FixedArray
ITKBasicFilters ITKCommon ITKIO)
