cmake_minimum_required(VERSION 2.6)

PROJECT(NeighborhoodOperatorImageFunction)

find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

ADD_EXECUTABLE(NeighborhoodOperatorImageFunction NeighborhoodOperatorImageFunction.cxx)
TARGET_LINK_LIBRARIES(NeighborhoodOperatorImageFunction ${ITK_LIBRARIES})
