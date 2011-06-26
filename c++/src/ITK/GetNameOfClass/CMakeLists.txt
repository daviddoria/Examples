cmake_minimum_required(VERSION 2.6)

PROJECT(GetNameOfClass)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(GetNameOfClass GetNameOfClass.cxx)
TARGET_LINK_LIBRARIES(GetNameOfClass ITKCommon)

