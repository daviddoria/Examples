cmake_minimum_required(VERSION 2.6)

PROJECT(ImageFunctionImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImageFunctionImageFilter ImageFunctionImageFilter.cxx)
TARGET_LINK_LIBRARIES(ImageFunctionImageFilter ${ITK_LIBRARIES})
