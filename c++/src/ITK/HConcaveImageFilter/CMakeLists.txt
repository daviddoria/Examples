cmake_minimum_required(VERSION 2.6)

PROJECT(HConcaveImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(HConcaveImageFilter HConcaveImageFilter.cxx)
TARGET_LINK_LIBRARIES(HConcaveImageFilter ${ITK_LIBRARIES})

