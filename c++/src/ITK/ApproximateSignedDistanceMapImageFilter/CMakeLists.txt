cmake_minimum_required(VERSION 2.6)

PROJECT(ApproximateSignedDistanceMapImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ApproximateSignedDistanceMapImageFilter ApproximateSignedDistanceMapImageFilter.cxx)
TARGET_LINK_LIBRARIES(ApproximateSignedDistanceMapImageFilter ${ITK_LIBRARIES})
