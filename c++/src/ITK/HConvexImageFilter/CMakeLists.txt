cmake_minimum_required(VERSION 2.6)

PROJECT(HConvexImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(HConvexImageFilter HConvexImageFilter.cxx)
TARGET_LINK_LIBRARIES(HConvexImageFilter ${ITK_LIBRARIES})

