cmake_minimum_required(VERSION 2.6)

PROJECT(ContourExtractor2DImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ContourExtractor2DImageFilter ContourExtractor2DImageFilter.cxx)
TARGET_LINK_LIBRARIES(ContourExtractor2DImageFilter ${ITK_LIBRARIES})
