cmake_minimum_required(VERSION 2.6)

PROJECT(ShrinkImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ShrinkImageFilter ShrinkImageFilter.cxx)
TARGET_LINK_LIBRARIES(ShrinkImageFilter ${ITK_LIBRARIES})
