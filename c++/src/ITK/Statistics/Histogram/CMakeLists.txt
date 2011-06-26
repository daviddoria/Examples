cmake_minimum_required(VERSION 2.6)

PROJECT(Histogram)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(Histogram Histogram.cxx)
TARGET_LINK_LIBRARIES(Histogram
ITKBasicFilters ITKCommon ITKIO ITKStatistics)
