cmake_minimum_required(VERSION 2.6)

PROJECT(MinimumImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(MinimumImageFilter MinimumImageFilter.cxx)
TARGET_LINK_LIBRARIES(MinimumImageFilter
ITKBasicFilters ITKCommon ITKIO)