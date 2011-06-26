cmake_minimum_required(VERSION 2.6)

PROJECT(ScalarToRGBPixelFunctor)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ScalarToRGBPixelFunctor ScalarToRGBPixelFunctor.cpp)
TARGET_LINK_LIBRARIES(ScalarToRGBPixelFunctor ${ITK_LIBRARIES})
