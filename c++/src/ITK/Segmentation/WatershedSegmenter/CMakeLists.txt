cmake_minimum_required(VERSION 2.6)

PROJECT(WatershedSegmenter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(WatershedSegmenter WatershedSegmenter.cpp)
TARGET_LINK_LIBRARIES(WatershedSegmenter ${ITK_LIBRARIES})
