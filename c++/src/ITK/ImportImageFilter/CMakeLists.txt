cmake_minimum_required(VERSION 2.6)

PROJECT(ImportImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImportImageFilter ImportImageFilter.cxx)
TARGET_LINK_LIBRARIES(ImportImageFilter ITKCommon ITKIO)

