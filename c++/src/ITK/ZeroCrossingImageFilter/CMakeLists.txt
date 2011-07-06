cmake_minimum_required(VERSION 2.6)

PROJECT(ZeroCrossingImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ZeroCrossingImageFilter ZeroCrossingImageFilter.cpp)
TARGET_LINK_LIBRARIES(ZeroCrossingImageFilter ${ITK_LIBRARIES})
