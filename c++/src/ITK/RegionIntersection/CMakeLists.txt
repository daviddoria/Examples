cmake_minimum_required(VERSION 2.6)

PROJECT(RegionIntersection)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(RegionIntersection RegionIntersection.cxx)
TARGET_LINK_LIBRARIES(RegionIntersection ITKCommon)