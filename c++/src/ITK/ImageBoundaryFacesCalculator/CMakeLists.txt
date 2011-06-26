cmake_minimum_required(VERSION 2.6)

PROJECT(ImageBoundaryFacesCalculator)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImageBoundaryFacesCalculator ImageBoundaryFacesCalculator.cxx)
TARGET_LINK_LIBRARIES(ImageBoundaryFacesCalculator 
ITKNumerics ITKBasicFilters ITKCommon ITKIO)
