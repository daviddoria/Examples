cmake_minimum_required(VERSION 2.6)

PROJECT(Optimization)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(Optimization Optimization.cpp)
TARGET_LINK_LIBRARIES(Optimization ITKCommon ITKNumerics)

