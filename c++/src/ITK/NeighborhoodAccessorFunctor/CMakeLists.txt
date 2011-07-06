cmake_minimum_required(VERSION 2.6)

PROJECT(NeighborhoodAccessorFunctor)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(NeighborhoodAccessorFunctor NeighborhoodAccessorFunctor.cxx)
TARGET_LINK_LIBRARIES(NeighborhoodAccessorFunctor ${ITK_LIBRARIES})
