cmake_minimum_required(VERSION 2.6)

PROJECT(RegionalMinimalImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(RegionalMinimalImageFilter RegionalMinimalImageFilter.cxx)
TARGET_LINK_LIBRARIES(RegionalMinimalImageFilter ${ITK_LIBRARIES})