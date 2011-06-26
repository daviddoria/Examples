cmake_minimum_required(VERSION 2.6)

PROJECT(VectorIndexSelectionCastImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(VectorIndexSelectionCastImageFilter VectorIndexSelectionCastImageFilter.cxx)
TARGET_LINK_LIBRARIES(VectorIndexSelectionCastImageFilter ${ITK_LIBRARIES})
