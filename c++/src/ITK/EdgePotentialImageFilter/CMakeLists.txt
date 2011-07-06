cmake_minimum_required(VERSION 2.6)

PROJECT(EdgePotentialImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(EdgePotentialImageFilter EdgePotentialImageFilter.cxx)
TARGET_LINK_LIBRARIES(EdgePotentialImageFilter ${ITK_LIBRARIES})
