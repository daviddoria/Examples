cmake_minimum_required(VERSION 2.6)

PROJECT(ComparisonInHeader)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ComparisonInHeader ComparisonInHeader.cpp)
TARGET_LINK_LIBRARIES(ComparisonInHeader ITKCommon)

