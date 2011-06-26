cmake_minimum_required(VERSION 2.6)

PROJECT(Transparency)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(Transparency Transparency.cxx)
TARGET_LINK_LIBRARIES(Transparency
ITKIO ITKBasicFilters ITKCommon )
