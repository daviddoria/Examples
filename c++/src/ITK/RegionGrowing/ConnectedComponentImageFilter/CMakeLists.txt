cmake_minimum_required(VERSION 2.6)

PROJECT(ConnectedComponentImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ConnectedComponentImageFilter ConnectedComponentImageFilter.cxx)
TARGET_LINK_LIBRARIES(ConnectedComponentImageFilter ${ITK_LIBRARIES})

