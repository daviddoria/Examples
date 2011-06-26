cmake_minimum_required(VERSION 2.6)

PROJECT(AzimuthElevationToCartesianTransform)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(AzimuthElevationToCartesianTransform AzimuthElevationToCartesianTransform.cxx)
TARGET_LINK_LIBRARIES(AzimuthElevationToCartesianTransform ${ITK_LIBRARIES})

