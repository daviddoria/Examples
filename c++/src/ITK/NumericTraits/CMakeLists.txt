cmake_minimum_required(VERSION 2.6)

PROJECT(NumericTraits)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(NumericTraits NumericTraits.cxx)
TARGET_LINK_LIBRARIES(NumericTraits
ITKBasicFilters ITKCommon ITKIO ITKStatistics)
