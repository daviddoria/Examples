cmake_minimum_required(VERSION 2.6)

PROJECT(TileImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(TileImageFilter TileImageFilter.cxx)
TARGET_LINK_LIBRARIES(TileImageFilter ${ITK_LIBRARIES})
