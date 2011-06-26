cmake_minimum_required(VERSION 2.6)

PROJECT(VectorConfidenceConnectedImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(VectorConfidenceConnectedImageFilter VectorConfidenceConnectedImageFilter.cxx)
TARGET_LINK_LIBRARIES(VectorConfidenceConnectedImageFilter ${ITK_LIBRARIES})

