cmake_minimum_required(VERSION 2.6)

PROJECT(SquareImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(SquareImageFilter SquareImageFilter.cxx)
TARGET_LINK_LIBRARIES(SquareImageFilter ${ITK_LIBRARIES})
