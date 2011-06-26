cmake_minimum_required(VERSION 2.6)

PROJECT(GaussianDistribution)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(GaussianDistribution GaussianDistribution.cxx)
TARGET_LINK_LIBRARIES(GaussianDistribution
ITKBasicFilters ITKCommon ITKIO ITKStatistics)
