cmake_minimum_required(VERSION 2.6)

PROJECT(MultiplyByConstantImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(MultiplyByConstantImageFilter MultiplyByConstantImageFilter.cxx)
TARGET_LINK_LIBRARIES(MultiplyByConstantImageFilter ITKIO)
