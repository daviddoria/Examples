cmake_minimum_required(VERSION 2.6)

PROJECT(CovariantVector)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(CovariantVector CovariantVector.cxx)
TARGET_LINK_LIBRARIES(CovariantVector ITKCommon)