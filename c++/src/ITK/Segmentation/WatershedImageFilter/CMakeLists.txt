cmake_minimum_required(VERSION 2.6)

PROJECT(WatershedImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(WatershedImageFilter WatershedImageFilter.cpp)
TARGET_LINK_LIBRARIES(WatershedImageFilter ${ITK_LIBRARIES})
