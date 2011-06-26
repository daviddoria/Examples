cmake_minimum_required(VERSION 2.6)

PROJECT(UnaryFunctorImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(UnaryFunctorImageFilter UnaryFunctorImageFilter.cxx)
TARGET_LINK_LIBRARIES(UnaryFunctorImageFilter
ITKIO ITKBasicFilters ITKCommon
)
