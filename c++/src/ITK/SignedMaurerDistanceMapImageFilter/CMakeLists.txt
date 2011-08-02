cmake_minimum_required(VERSION 2.6)

PROJECT(SignedMaurerDistanceMapImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(SignedMaurerDistanceMapImageFilter SignedMaurerDistanceMapImageFilter.cxx)
TARGET_LINK_LIBRARIES(SignedMaurerDistanceMapImageFilter ${ITK_LIBRARIES})
