cmake_minimum_required(VERSION 2.6)

PROJECT(SumOfSquaredDifferences)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(SumOfSquaredDifferences SumOfSquaredDifferences.cxx)
TARGET_LINK_LIBRARIES(SumOfSquaredDifferences
ITKBasicFilters ITKCommon ITKIO)