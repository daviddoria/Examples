cmake_minimum_required(VERSION 2.6)

PROJECT(BinaryFunctorImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(BinaryFunctorImageFilter BinaryFunctorImageFilter.cxx)
TARGET_LINK_LIBRARIES(BinaryFunctorImageFilter ${ITK_LIBRARIES})
