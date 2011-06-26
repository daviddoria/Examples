cmake_minimum_required(VERSION 2.6)

PROJECT(NearestNeighborInterpolateImageFunction)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(NearestNeighborInterpolateImageFunction NearestNeighborInterpolateImageFunction.cxx)
TARGET_LINK_LIBRARIES(NearestNeighborInterpolateImageFunction ${ITK_LIBRARIES})

