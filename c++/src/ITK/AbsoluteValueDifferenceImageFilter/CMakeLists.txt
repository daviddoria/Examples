cmake_minimum_required(VERSION 2.6)

PROJECT(AbsoluteValueDifferenceImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(AbsoluteValueDifferenceImageFilter AbsoluteValueDifferenceImageFilter.cxx)
TARGET_LINK_LIBRARIES(AbsoluteValueDifferenceImageFilter ${ITK_LIBRARIES})