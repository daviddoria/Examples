cmake_minimum_required(VERSION 2.6)

PROJECT(HammingWindowFunction)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(HammingWindowFunction HammingWindowFunction.cxx)
TARGET_LINK_LIBRARIES(HammingWindowFunction ITKCommon)

