cmake_minimum_required(VERSION 2.6)

PROJECT(ImageRandomConstIteratorWithIndex)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImageRandomConstIteratorWithIndex ImageRandomConstIteratorWithIndex.cxx)
TARGET_LINK_LIBRARIES(ImageRandomConstIteratorWithIndex ITKNumerics ITKBasicFilters ITKCommon ITKIO)
