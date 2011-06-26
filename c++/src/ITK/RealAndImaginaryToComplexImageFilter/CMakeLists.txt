cmake_minimum_required(VERSION 2.6)

PROJECT(RealAndImaginaryToComplexImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(RealAndImaginaryToComplexImageFilter RealAndImaginaryToComplexImageFilter.cxx)
TARGET_LINK_LIBRARIES(RealAndImaginaryToComplexImageFilter ITKIO)
