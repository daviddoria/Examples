cmake_minimum_required(VERSION 2.6)

PROJECT(VectorImageResampleImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(VectorImageResampleImageFilter VectorImageResampleImageFilter.cxx)
TARGET_LINK_LIBRARIES(VectorImageResampleImageFilter ${ITK_LIBRARIES})

