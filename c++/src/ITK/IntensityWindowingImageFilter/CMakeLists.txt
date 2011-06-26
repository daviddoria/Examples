cmake_minimum_required(VERSION 2.6)

PROJECT(IntensityWindowingImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(IntensityWindowingImageFilter IntensityWindowingImageFilter.cxx)
TARGET_LINK_LIBRARIES(IntensityWindowingImageFilter
ITKBasicFilters ITKCommon ITKIO)
