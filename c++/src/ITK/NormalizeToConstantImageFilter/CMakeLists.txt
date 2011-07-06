cmake_minimum_required(VERSION 2.6)

PROJECT(NormalizeToConstantImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(NormalizeToConstantImageFilter NormalizeToConstantImageFilter.cpp)
TARGET_LINK_LIBRARIES(NormalizeToConstantImageFilter ${ITK_LIBRARIES})
