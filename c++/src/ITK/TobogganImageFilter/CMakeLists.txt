cmake_minimum_required(VERSION 2.6)

PROJECT(TobogganImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(TobogganImageFilter TobogganImageFilter.cpp)
TARGET_LINK_LIBRARIES(TobogganImageFilter ${ITK_LIBRARIES})
