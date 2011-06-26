cmake_minimum_required(VERSION 2.6)

PROJECT(TestingComparisonImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(TestingComparisonImageFilter TestingComparisonImageFilter.cpp)
TARGET_LINK_LIBRARIES(TestingComparisonImageFilter ${ITK_LIBRARIES})

