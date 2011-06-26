cmake_minimum_required(VERSION 2.6)

PROJECT(BinaryNotImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(BinaryNotImageFilter BinaryNotImageFilter.cxx)
TARGET_LINK_LIBRARIES(BinaryNotImageFilter ITKIO)

