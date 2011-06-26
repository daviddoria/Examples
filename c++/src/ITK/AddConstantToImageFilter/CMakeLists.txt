cmake_minimum_required(VERSION 2.6)

PROJECT(AddConstantToImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(AddConstantToImageFilter AddConstantToImageFilter.cxx)
TARGET_LINK_LIBRARIES(AddConstantToImageFilter ${ITK_LIBRARIES})
