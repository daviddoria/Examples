cmake_minimum_required(VERSION 2.6)

PROJECT(GradientToMagnitudeImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(GradientToMagnitudeImageFilter GradientToMagnitudeImageFilter.cxx)
TARGET_LINK_LIBRARIES(GradientToMagnitudeImageFilter ${ITK_LIBRARIES})
