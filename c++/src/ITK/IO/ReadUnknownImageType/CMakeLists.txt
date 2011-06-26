cmake_minimum_required(VERSION 2.6)

PROJECT(ReadUnknownImageType)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ReadUnknownImageType ReadUnknownImageType.cxx)
TARGET_LINK_LIBRARIES(ReadUnknownImageType ITKIO ITKBasicFilters ITKCommon )

ADD_EXECUTABLE(CreateImages CreateImages.cxx)
TARGET_LINK_LIBRARIES(CreateImages ITKIO ITKBasicFilters ITKCommon )