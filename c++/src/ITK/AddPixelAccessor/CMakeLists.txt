cmake_minimum_required(VERSION 2.6)

PROJECT(AddPixelAccessor)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(AddPixelAccessor AddPixelAccessor.cxx)
TARGET_LINK_LIBRARIES(AddPixelAccessor
ITKBasicFilters ITKCommon ITKIO)