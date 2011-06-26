cmake_minimum_required(VERSION 2.6)

PROJECT(NearestNeighborExtrapolateImageFunction)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(NearestNeighborExtrapolateImageFunction NearestNeighborExtrapolateImageFunction.cxx)
TARGET_LINK_LIBRARIES(NearestNeighborExtrapolateImageFunction ${ITK_LIBRARIES})

