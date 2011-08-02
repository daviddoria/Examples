cmake_minimum_required(VERSION 2.6)

PROJECT(SignedDanielssonDistanceMapImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(SignedDanielssonDistanceMapImageFilter SignedDanielssonDistanceMapImageFilter.cxx)
TARGET_LINK_LIBRARIES(SignedDanielssonDistanceMapImageFilter ${ITK_LIBRARIES})
