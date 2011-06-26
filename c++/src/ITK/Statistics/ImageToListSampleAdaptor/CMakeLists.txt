cmake_minimum_required(VERSION 2.6)

PROJECT(ImageToListSampleAdaptor)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImageToListSampleAdaptor ImageToListSampleAdaptor.cxx)
TARGET_LINK_LIBRARIES(ImageToListSampleAdaptor
ITKBasicFilters ITKCommon ITKIO)
