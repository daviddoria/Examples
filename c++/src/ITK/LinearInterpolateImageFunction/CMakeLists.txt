cmake_minimum_required(VERSION 2.6)

PROJECT(LinearInterpolateImageFunction)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(LinearInterpolateImageFunction LinearInterpolateImageFunction.cxx)
TARGET_LINK_LIBRARIES(LinearInterpolateImageFunction ${ITK_LIBRARIES})

