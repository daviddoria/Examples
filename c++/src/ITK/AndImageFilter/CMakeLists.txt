cmake_minimum_required(VERSION 2.6)

PROJECT(AndImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(AndImageFilter AndImageFilter.cxx)
TARGET_LINK_LIBRARIES(AndImageFilter ITKIO)

