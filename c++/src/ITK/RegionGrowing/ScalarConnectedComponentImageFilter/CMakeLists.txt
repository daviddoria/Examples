cmake_minimum_required(VERSION 2.6)

PROJECT(ScalarConnectedComponentImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ScalarConnectedComponentImageFilter ScalarConnectedComponentImageFilter.cxx)
TARGET_LINK_LIBRARIES(ScalarConnectedComponentImageFilter ${ITK_LIBRARIES})
