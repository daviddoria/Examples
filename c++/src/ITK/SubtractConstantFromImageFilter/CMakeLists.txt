cmake_minimum_required(VERSION 2.6)

PROJECT(SubtractConstantFromImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(SubtractConstantFromImageFilter SubtractConstantFromImageFilter.cxx)
TARGET_LINK_LIBRARIES(SubtractConstantFromImageFilter ${ITK_LIBRARIES})
