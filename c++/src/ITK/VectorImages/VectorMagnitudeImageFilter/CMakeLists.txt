cmake_minimum_required(VERSION 2.6)

PROJECT(VectorMagnitudeImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(VectorMagnitudeImageFilter VectorMagnitudeImageFilter.cxx)
TARGET_LINK_LIBRARIES(VectorMagnitudeImageFilter ${ITK_LIBRARIES})
