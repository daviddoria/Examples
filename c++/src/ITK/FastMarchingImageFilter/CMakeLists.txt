cmake_minimum_required(VERSION 2.6)

PROJECT(FastMarchingImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(FastMarchingImageFilter FastMarchingImageFilter.cxx)
TARGET_LINK_LIBRARIES(FastMarchingImageFilter
ITKIO ITKBasicFilters ITKCommon
)
