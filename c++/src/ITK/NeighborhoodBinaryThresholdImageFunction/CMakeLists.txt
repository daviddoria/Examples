cmake_minimum_required(VERSION 2.6)

PROJECT(NeighborhoodBinaryThresholdImageFunction)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(NeighborhoodBinaryThresholdImageFunction NeighborhoodBinaryThresholdImageFunction.cxx)
TARGET_LINK_LIBRARIES(NeighborhoodBinaryThresholdImageFunction ${ITK_LIBRARIES})
