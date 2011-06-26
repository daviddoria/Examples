cmake_minimum_required(VERSION 2.6)

PROJECT(BlackTopHatImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(BlackTopHatImageFilter BlackTopHatImageFilter.cxx)
TARGET_LINK_LIBRARIES(BlackTopHatImageFilter ${ITK_LIBRARIES})

