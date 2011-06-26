cmake_minimum_required(VERSION 2.6)

PROJECT(VectorResampleImageFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(VectorResampleImageFilter VectorResampleImageFilter.cxx)
TARGET_LINK_LIBRARIES(VectorResampleImageFilter vtkHybrid
ITKIO ITKNumerics ITKBasicFilters ITKCommon
)

