cmake_minimum_required(VERSION 2.6)

PROJECT(Distance)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(Distance Distance.cpp)
TARGET_LINK_LIBRARIES(Distance ITKNumerics)

