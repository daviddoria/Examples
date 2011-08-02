cmake_minimum_required(VERSION 2.6)

PROJECT(LabelStatisticsImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(LabelStatisticsImageFilter LabelStatisticsImageFilter.cxx)
TARGET_LINK_LIBRARIES(LabelStatisticsImageFilter ${ITK_LIBRARIES} )
