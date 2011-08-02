cmake_minimum_required(VERSION 2.6)

PROJECT(MorphologicalWatershedImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(MorphologicalWatershedImageFilter MorphologicalWatershedImageFilter.cpp)
TARGET_LINK_LIBRARIES(MorphologicalWatershedImageFilter ${ITK_LIBRARIES})
