cmake_minimum_required(VERSION 2.6)

PROJECT(GradientOfVectorImage)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(GradientOfVectorImage GradientOfVectorImage.cxx)
TARGET_LINK_LIBRARIES(GradientOfVectorImage ITKBasicFilters ITKIO ITKCommon)
