cmake_minimum_required(VERSION 2.6)

PROJECT(CastImageFilterVector)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(CastImageFilterVector CastImageFilterVector.cxx)
TARGET_LINK_LIBRARIES(CastImageFilterVector
ITKIO ITKBasicFilters ITKCommon)