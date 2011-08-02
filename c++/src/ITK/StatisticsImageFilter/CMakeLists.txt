cmake_minimum_required(VERSION 2.6)

PROJECT(StatisticsImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(StatisticsImageFilter StatisticsImageFilter.cxx)
TARGET_LINK_LIBRARIES(StatisticsImageFilter ${ITK_LIBRARIES})