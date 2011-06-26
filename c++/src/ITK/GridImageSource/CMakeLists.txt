cmake_minimum_required(VERSION 2.6)

PROJECT(GridImageSource)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(GridImageSource GridImageSource.cxx)
TARGET_LINK_LIBRARIES(GridImageSource
ITKIO ITKBasicFilters ITKCommon )
