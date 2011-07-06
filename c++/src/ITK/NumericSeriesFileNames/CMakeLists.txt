cmake_minimum_required(VERSION 2.6)

PROJECT(NumericSeriesFileNames)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(NumericSeriesFileNames NumericSeriesFileNames.cxx)
TARGET_LINK_LIBRARIES(NumericSeriesFileNames ${ITK_LIBRARIES})
