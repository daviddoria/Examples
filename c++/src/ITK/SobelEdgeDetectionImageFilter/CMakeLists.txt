cmake_minimum_required(VERSION 2.6)

PROJECT(SobelEdgeDetectionImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(SobelEdgeDetectionImageFilter SobelEdgeDetectionImageFilter.cxx)
TARGET_LINK_LIBRARIES(SobelEdgeDetectionImageFilter ${ITK_LIBRARIES})
