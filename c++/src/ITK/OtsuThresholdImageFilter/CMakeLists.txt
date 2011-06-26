cmake_minimum_required(VERSION 2.6)

PROJECT(OtsuThresholdImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(OtsuThresholdImageFilter OtsuThresholdImageFilter.cxx)
TARGET_LINK_LIBRARIES(OtsuThresholdImageFilter
ITKBasicFilters ITKCommon ITKIO)
