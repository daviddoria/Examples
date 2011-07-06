cmake_minimum_required(VERSION 2.6)

PROJECT(OtsuMultipleThresholdCalculator)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(OtsuMultipleThresholdCalculator OtsuMultipleThresholdCalculator.cxx)
TARGET_LINK_LIBRARIES(OtsuMultipleThresholdCalculator ${ITK_LIBRARIES})
