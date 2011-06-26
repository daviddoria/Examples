cmake_minimum_required(VERSION 2.6)

PROJECT(GaussianMixtureModelComponent)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(GaussianMixtureModelComponent GaussianMixtureModelComponent.cxx)
TARGET_LINK_LIBRARIES(GaussianMixtureModelComponent
ITKBasicFilters ITKCommon ITKIO ITKStatistics)

