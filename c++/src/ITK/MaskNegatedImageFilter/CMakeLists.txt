cmake_minimum_required(VERSION 2.6)

PROJECT(MaskNegatedImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(MaskNegatedImageFilter MaskNegatedImageFilter.cxx)
TARGET_LINK_LIBRARIES(MaskNegatedImageFilter ${ITK_LIBRARIES})

