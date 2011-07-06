cmake_minimum_required(VERSION 2.6)

PROJECT(WarpImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(WarpImageFilter WarpImageFilter.cxx)
TARGET_LINK_LIBRARIES(WarpImageFilter ${ITK_LIBRARIES})
