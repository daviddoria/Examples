cmake_minimum_required(VERSION 2.6)

PROJECT(MaskNeighborhoodOperatorImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(MaskNeighborhoodOperatorImageFilter MaskNeighborhoodOperatorImageFilter.cxx)
TARGET_LINK_LIBRARIES(MaskNeighborhoodOperatorImageFilter ${ITK_LIBRARIES})
