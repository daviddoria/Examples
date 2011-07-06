cmake_minimum_required(VERSION 2.6)

PROJECT(GaussianBlurImageFunction)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(GaussianBlurImageFunction GaussianBlurImageFunction.cxx)
TARGET_LINK_LIBRARIES(GaussianBlurImageFunction ${ITK_LIBRARIES})
