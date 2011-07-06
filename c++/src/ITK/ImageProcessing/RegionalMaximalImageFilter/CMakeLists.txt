cmake_minimum_required(VERSION 2.6)

PROJECT(RegionalMaximalImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(RegionalMaximalImageFilter RegionalMaximalImageFilter.cxx)
TARGET_LINK_LIBRARIES(RegionalMaximalImageFilter
ITKBasicFilters ITKCommon ITKIO)