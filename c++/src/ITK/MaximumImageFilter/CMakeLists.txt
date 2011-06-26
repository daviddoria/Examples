cmake_minimum_required(VERSION 2.6)

PROJECT(MaximumImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(MaximumImageFilter MaximumImageFilter.cxx)
TARGET_LINK_LIBRARIES(MaximumImageFilter
ITKBasicFilters ITKCommon ITKIO)