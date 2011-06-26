cmake_minimum_required(VERSION 2.6)

PROJECT(JoinImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(JoinImageFilter JoinImageFilter.cxx)
TARGET_LINK_LIBRARIES(JoinImageFilter ${ITK_LIBRARIES})