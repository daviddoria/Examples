cmake_minimum_required(VERSION 2.6)

PROJECT(ExtractImageFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ExtractImageFilter ExtractImageFilter.cpp)
TARGET_LINK_LIBRARIES(ExtractImageFilter
vtkHybrid
ITKBasicFilters ITKCommon ITKIO)
