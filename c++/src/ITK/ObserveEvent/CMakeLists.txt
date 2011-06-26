cmake_minimum_required(VERSION 2.6)

PROJECT(ObserveEvent)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ObserveEvent ObserveEvent.cpp)
TARGET_LINK_LIBRARIES(ObserveEvent ITKCommon)

