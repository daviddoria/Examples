cmake_minimum_required(VERSION 2.6)

PROJECT(ImageSource)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImageSource ImageSourceExample.cxx)
TARGET_LINK_LIBRARIES(ImageSource
ITKBasicFilters ITKIO ITKCommon)
