cmake_minimum_required(VERSION 2.6)

PROJECT(SimpleContourExtractorImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(SimpleContourExtractorImageFilter SimpleContourExtractorImageFilter.cxx)
TARGET_LINK_LIBRARIES(SimpleContourExtractorImageFilter
ITKBasicFilters ITKCommon ITKIO)
