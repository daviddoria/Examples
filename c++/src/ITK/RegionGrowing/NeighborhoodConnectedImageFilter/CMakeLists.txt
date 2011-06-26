cmake_minimum_required(VERSION 2.6)

PROJECT(NeighborhoodConnectedImageFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(NeighborhoodConnectedImageFilter NeighborhoodConnectedImageFilter.cxx)
TARGET_LINK_LIBRARIES(NeighborhoodConnectedImageFilter ${ITK_LIBRARIES})
