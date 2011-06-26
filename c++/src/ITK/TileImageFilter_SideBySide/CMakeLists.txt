cmake_minimum_required(VERSION 2.6)

PROJECT(TileImageFilter_SideBySide)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(TileImageFilter_SideBySide TileImageFilter_SideBySide.cxx)
TARGET_LINK_LIBRARIES(TileImageFilter_SideBySide ${ITK_LIBRARIES})
