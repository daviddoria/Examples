cmake_minimum_required(VERSION 2.6)

PROJECT(DanielssonDistanceMapImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(DanielssonDistanceMapImageFilter DanielssonDistanceMapImageFilter.cxx)
TARGET_LINK_LIBRARIES(DanielssonDistanceMapImageFilter ${ITK_LIBRARIES})
