cmake_minimum_required(VERSION 2.6)

PROJECT(ListSample)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ListSample ListSample.cxx)
TARGET_LINK_LIBRARIES(ListSample
ITKBasicFilters ITKCommon ITKIO)
