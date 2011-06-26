cmake_minimum_required(VERSION 2.6)

PROJECT(RandomPermutation)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(RandomPermutation RandomPermutation.cxx)
TARGET_LINK_LIBRARIES(RandomPermutation ITKCommon)

