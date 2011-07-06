cmake_minimum_required(VERSION 2.6)

PROJECT(MaskedImageToHistogramFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(MaskedImageToHistogramFilter MaskedImageToHistogramFilter.cxx)
TARGET_LINK_LIBRARIES(MaskedImageToHistogramFilter ${ITK_LIBRARIES})
