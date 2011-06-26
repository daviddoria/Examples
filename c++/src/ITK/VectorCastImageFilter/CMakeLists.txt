cmake_minimum_required(VERSION 2.6)

PROJECT(VectorCastImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(VectorCastImageFilter VectorCastImageFilter.cxx)
TARGET_LINK_LIBRARIES(VectorCastImageFilter ${ITK_LIBRARIES})